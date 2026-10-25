// H1 slice 6: cvc::gl::ariadne::room_view — the conference room in 3D, offscreen. The room scene
// from the replicated tree is realized (and re-realized on a revision change), avatar markers
// follow a client holon's participants and poses, a local reconcile jump is eased (no visible
// step), the key -> intent mapping walks and strafes relative to the avatar's facing, the visor
// turns with the facing, a pixel picks the ground point under it, a participant whose pose is not
// known yet has no marker (and the local avatar appears at its first pose without gliding in), the
// room's mesh objects are LOD ladders whose rung follows the camera every frame (one build for one
// asset, kept across a re-realize; the default room has none), and teardown removes every node
// room_view made.
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/ariadne/avatar_sim.h>
#include <cvc/ariadne/conference/room_scene.h>
#include <cvc/ariadne/holon.h>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/LodGraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/room_view.h>
#include <cvc/lod/select.h>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <vtkCamera.h>
#include <vtkMatrix4x4.h>
#include <vtkRenderer.h>

using cvc::ariadne::avatar_intent;
using cvc::ariadne::holon;
using cvc::ariadne::holon_role;
using cvc::gl::LodGraphicsNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::ariadne::heading_yaw;
using cvc::gl::ariadne::intent_from_keys;
using cvc::gl::ariadne::room_view;
using cvc::physics::body_state;
using cvc::physics::vec3;
namespace rs = cvc::ariadne::conference;

static int fails = 0;
static void chk(bool ok, const std::string &w) {
  printf("  %s  %s\n", ok ? "PASS" : "FAIL", w.c_str());
  if (!ok)
    ++fails;
}
static bool near(double a, double b, double tol = 1e-6) { return std::fabs(a - b) < tol; }

static bool node_xyz(SceneGraph &sg, const std::string &id, double out[3]) {
  std::shared_ptr<cvc::gl::GraphicsNode> n = sg.getGraphics(id);
  if (!n)
    return false;
  const vtkMatrix4x4 *m = n->getTransform();
  out[0] = m->GetElement(0, 3), out[1] = m->GetElement(1, 3), out[2] = m->GetElement(2, 3);
  return true;
}

static std::shared_ptr<LodGraphicsNode> lod_node(SceneGraph &sg, const std::string &id) {
  return std::dynamic_pointer_cast<LodGraphicsNode>(sg.getGraphics(id));
}

// Aim the view's camera (what room_view's LOD selection reads) from EYE at FOCAL, Z up.
static void look(SceneRenderer &view, const double eye[3], const double focal[3]) {
  vtkCamera *cam = view.renderer()->GetActiveCamera();
  cam->SetPosition(eye[0], eye[1], eye[2]);
  cam->SetFocalPoint(focal[0], focal[1], focal[2]);
  cam->SetViewUp(0.0, 0.0, 1.0);
}

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");

  // ── keys -> intent, relative to the avatar's facing ──────────────────────
  {
    const double kPi = 3.14159265358979323846;
    avatar_intent w = intent_from_keys(true, false, false, false, 0.0, 2.0); // facing +x
    chk(near(w.move.x, 2.0) && near(w.move.y, 0.0) && near(w.move.z, 0.0) && near(w.yaw, 0.0),
        "W walks along the facing, and the intent carries it");
    avatar_intent d = intent_from_keys(false, false, false, true, 0.0, 2.0);
    chk(near(d.move.x, 0.0) && near(d.move.y, -2.0), "D strafes right of the facing (-y here)");
    avatar_intent s = intent_from_keys(false, true, false, false, 0.5 * kPi, 2.0); // facing +y
    chk(near(s.move.x, 0.0) && near(s.move.y, -2.0) && near(s.yaw, 0.5 * kPi),
        "S backs away, still facing the same way");
    avatar_intent a = intent_from_keys(false, false, true, false, 0.5 * kPi, 2.0);
    chk(near(a.move.x, -2.0) && near(a.move.y, 0.0), "A strafes left of the facing (-x here)");
    avatar_intent wd = intent_from_keys(true, false, false, true, 0.0, 2.0);
    chk(near(std::hypot(wd.move.x, wd.move.y), 2.0) && wd.move.x > 0 && wd.move.y < 0,
        "a diagonal is normalized, not faster");
    avatar_intent ws = intent_from_keys(true, true, false, false, 1.0, 2.0);
    chk(near(ws.move.x, 0.0) && near(ws.move.y, 0.0) && near(ws.yaw, 1.0),
        "opposing keys cancel, and standing still keeps the facing");
    const double eye[3] = {-10, 0, 5}, focal[3] = {0, 0, 0};
    chk(near(heading_yaw(eye, focal), 0.0), "heading_yaw: the view along +x is yaw 0");
    const double top[3] = {0, 0, 10}, down[3] = {0, 0, 0};
    chk(near(heading_yaw(top, down), 0.5 * kPi), "heading_yaw: straight down is +Y");
  }

  // ── the facing reaches the avatar's rotation ─────────────────────────────
  {
    cvc::ariadne::avatar_sim sim;
    avatar_intent in;
    in.yaw = 0.5 * 3.14159265358979323846;
    sim.step(in, 1.0 / 60.0);
    const cvc::physics::quat q = sim.state().rotation;
    chk(near(q.w, std::sqrt(0.5)) && near(q.z, std::sqrt(0.5)) && near(q.x, 0.0) && near(q.y, 0.0),
        "the sim faces the intent's yaw (a turn about +Z)");
  }

  // ── the camera's held keys are what the host reads for the avatar ─────────
  {
    cvc::gl::CameraController c(app, "room_keys.camera");
    c.keyDown("W");
    c.keyDown("Up");
    chk(c.keyHeld("w") && c.keyHeld("W") && c.keyHeld("Up"), "keyHeld sees held keys, any case");
    c.keyUp("w");
    chk(!c.keyHeld("w") && c.keyHeld("Up"), "and their release");
    c.releaseHeldKeys(); // what the UI overlay does when it takes the keyboard
    chk(!c.keyHeld("Up"), "and the drop when a UI takes the keyboard");
  }

  // ── the room scene: realized, then re-realized on a revision change ───────
  {
    SceneGraph sg(app, "room_scene");
    SceneRenderer view(sg, 128, 128, /*offscreen=*/true, "main");
    holon h(holon_role::client);
    rs::seed_default_scene(app, "r");
    // The default room has no lights (its rig is the client document's); a room MAY carry its own,
    // and room_view realizes them like the objects.
    {
      rs::room_scene_env env = rs::default_scene_env();
      cvc::ariadne::SceneLight sun;
      sun.id = "sun";
      env.lights.push_back(sun);
      rs::write_scene_env(app, "r", env);
    }
    {
      room_view rv(view, app, "r", "A", sg.getStatePrefix());
      rv.update(h, 1.0 / 60.0);
      chk(rv.realized_revision() == rs::scene_revision(app, "r"), "realized the current revision");
      chk(sg.getGraphics("room_ground") != nullptr, "the ground is in the scene graph");
      chk(sg.getGraphics("room_ref") != nullptr, "the reference box is in the scene graph");
      chk(sg.getGraphics("room_sun") != nullptr, "the sun light is in the scene graph");
      double ref[3];
      chk(node_xyz(sg, "room_ref", ref) && near(ref[2], 0.5, 1e-4), "the box stands on the ground");

      rs::remove_scene_object(app, "r", "ref");
      rv.update(h, 1.0 / 60.0);
      chk(sg.getGraphics("room_ref") == nullptr, "a removed object leaves the graph");
      chk(sg.getGraphics("room_ground") != nullptr, "the rest is re-realized");
      chk(sg.getGraphics("room_sun") != nullptr, "the light too (not duplicated or lost)");
    }
    chk(sg.getGraphics("room_ground") == nullptr && sg.getGraphics("room_sun") == nullptr,
        "teardown removes the room's nodes and lights");
  }

  // ── avatars follow the holon; a local correction is eased ────────────────
  {
    SceneGraph sg(app, "room_avatars");
    SceneRenderer view(sg, 128, 128, /*offscreen=*/true, "main");
    holon h(holon_role::client);
    body_state a0, b0;
    a0.position = vec3{4.0, 0.0, 0.0};
    b0.position = vec3{0.0, 4.0, 0.0};
    h.add_participant("A", /*local=*/true, a0);
    h.add_participant("B", /*local=*/false, b0);
    const double dt = h.clock().fixed_dt();
    {
      room_view rv(view, app, "empty", "A", sg.getStatePrefix());
      h.advance(dt);
      rv.update(h, dt);
      double pa[3], pb[3], self[3];
      chk(node_xyz(sg, "avatar_A", pa) && near(pa[0], 4.0) && near(pa[2], 0.85, 1e-4),
          "the local avatar's marker stands at its pose");
      chk(node_xyz(sg, "avatar_B", pb) && near(pb[1], 4.0), "a peer's marker stands at its pose");
      chk(rv.self_position(self) && near(self[0], 4.0), "self_position is the local avatar");
      double va[3];
      chk(node_xyz(sg, "avatar_A_visor", va) && near(va[0], 4.25, 1e-4) && near(va[1], 0.0) &&
              va[2] > 1.0,
          "the visor is on the front (+x at yaw 0), at head height");

      // Pick: aim the camera straight down at (4, 0) and the centre pixel is that ground point.
      vtkCamera *cam = view.renderer()->GetActiveCamera();
      cam->SetPosition(4.0, 0.0, 20.0);
      cam->SetFocalPoint(4.0, 0.0, 0.0);
      cam->SetViewUp(0.0, 1.0, 0.0);
      view.renderer()->ResetCameraClippingRange();
      int *vs = view.renderer()->GetSize();
      double g[3];
      chk(rv.pick_ground(vs[0] / 2, vs[1] / 2, 0.0, g) && near(g[0], 4.0, 0.2) &&
              near(g[1], 0.0, 0.2) && near(g[2], 0.0),
          "pick_ground: the centre pixel lands under the camera");
      double g2[3];
      chk(rv.pick_ground(vs[0] - 1, vs[1] / 2, 0.0, g2) && g2[0] > g[0] + 0.5,
          "pick_ground: a pixel to the right lands further along +x");

      // Turn A to face +y: the visor swings round to its +y side.
      avatar_intent turn;
      turn.yaw = 0.5 * 3.14159265358979323846;
      h.set_intent("A", turn);
      h.advance(dt);
      rv.update(h, dt);
      chk(node_xyz(sg, "avatar_A_visor", va) && near(va[0], 4.0, 1e-4) && near(va[1], 0.25, 1e-4),
          "the visor turns with the facing");

      // A server correction teleports A by +3 m in y: the marker must not jump this frame,
      // then glides to the corrected pose.
      double before[3];
      node_xyz(sg, "avatar_A", before);
      body_state moved = a0;
      moved.position.y = 3.0;
      h.apply_authoritative("A", moved, h.tick() + 1);
      h.advance(dt);
      rv.update(h, dt);
      double now[3];
      node_xyz(sg, "avatar_A", now);
      // One frame's share of the 3 m correction (3 * (1 - exp(-dt / tau)) ~ 0.2 m), not the jump.
      const double share = 3.0 * (1.0 - std::exp(-dt / rv.correction_tau));
      chk(near(now[1] - before[1], share, 1e-6), "a correction does not jump the marker");
      for (int i = 0; i < 120; ++i) { // 1 s >> correction_tau
        h.advance(dt);
        rv.update(h, dt);
      }
      node_xyz(sg, "avatar_A", now);
      chk(near(now[1], 3.0, 1e-3), "and the marker glides onto the corrected pose");

      h.remove_participant("B");
      h.advance(dt);
      rv.update(h, dt);
      chk(sg.getGraphics("avatar_B") == nullptr && sg.getGraphics("avatar_B_visor") == nullptr,
          "a departed peer's marker (and visor) is removed");
      chk(sg.getGraphics("avatar_A") != nullptr, "the others stay");
    }
    chk(sg.getGraphics("avatar_A") == nullptr && sg.getGraphics("avatar_A_visor") == nullptr,
        "teardown removes the avatar markers");
  }

  // ── no marker for a peer whose pose is not known yet ──────────────────────
  {
    SceneGraph sg(app, "room_pose_unknown");
    SceneRenderer view(sg, 128, 128, /*offscreen=*/true, "main");
    holon h(holon_role::client);
    body_state a0;
    a0.position = vec3{4.0, 0.0, 0.0};
    h.add_participant("A", /*local=*/true, a0);
    // In the roster before its first sample (client_pump): its start is a placeholder origin.
    h.add_participant("C", /*local=*/false, body_state{}, /*pose_known=*/false);
    const double dt = h.clock().fixed_dt();
    {
      room_view rv(view, app, "empty", "A", sg.getStatePrefix());
      h.advance(dt);
      rv.update(h, dt);
      chk(sg.getGraphics("avatar_C") == nullptr && sg.getGraphics("avatar_C_visor") == nullptr,
          "no marker for a peer whose pose is not known (not one at the origin)");
      const std::vector<std::string> shown = rv.avatar_pids();
      chk(shown.size() == 1 && shown[0] == "A", "only the participants with a pose are drawn");

      body_state c1;
      c1.position = vec3{-3.0, 2.0, 0.0};
      h.apply_authoritative("C", c1, h.tick());
      h.advance(dt);
      rv.update(h, dt);
      double pc[3];
      chk(node_xyz(sg, "avatar_C", pc) && near(pc[0], -3.0) && near(pc[1], 2.0) &&
              sg.getGraphics("avatar_C_visor") != nullptr,
          "its marker appears at its first sample");
    }
    chk(sg.getGraphics("avatar_C") == nullptr, "teardown removes it too");
  }

  // ── the local avatar appears at its first pose, without a glide ───────────
  {
    SceneGraph sg(app, "room_self_first");
    SceneRenderer view(sg, 128, 128, /*offscreen=*/true, "main");
    holon h(holon_role::client);
    h.add_participant("A", /*local=*/true, body_state{}, /*pose_known=*/false);
    const double dt = h.clock().fixed_dt();
    {
      room_view rv(view, app, "empty", "A", sg.getStatePrefix());
      h.advance(dt);
      rv.update(h, dt);
      double self[3];
      chk(sg.getGraphics("avatar_A") == nullptr && !rv.self_position(self),
          "no self marker (and no self_position) before the first pose");

      // The first sample, from a server that has run a while: the replica re-aligns its clock and
      // snaps the avatar from the origin to it -- a reconcile jump, but not one to ease out.
      body_state first;
      first.position = vec3{2.0, -3.0, 0.0};
      h.apply_authoritative("A", first, 500);
      h.advance(dt);
      rv.update(h, dt);
      double pa[3];
      chk(node_xyz(sg, "avatar_A", pa) && near(pa[0], 2.0) && near(pa[1], -3.0),
          "the self marker appears at its first pose, not on its way from the origin");
      chk(rv.self_position(self) && near(self[0], 2.0) && near(self[1], -3.0),
          "self_position is there from the start");
      for (int i = 0; i < 30; ++i) {
        h.advance(dt);
        rv.update(h, dt);
      }
      chk(node_xyz(sg, "avatar_A", pa) && near(pa[0], 2.0) && near(pa[1], -3.0),
          "and it stays there (no glide)");
    }
  }

  // ── the default room has no LOD objects ───────────────────────────────────
  {
    SceneGraph sg(app, "room_lod_default");
    SceneRenderer view(sg, 128, 128, /*offscreen=*/true, "main");
    holon h(holon_role::client);
    rs::seed_default_scene(app, "lod_default");
    const double eye[3] = {0.0, -10.0, 2.0}, focal[3] = {0.0, 0.0, 0.0};
    look(view, eye, focal); // a camera to select from: what keeps the stats at zero is the room
    {
      room_view rv(view, app, "lod_default", "A", sg.getStatePrefix());
      rv.update(h, 1.0 / 60.0);
      chk(sg.getGraphics("room_ground") && sg.getGraphics("room_ref") &&
              sg.getAllGraphicsOfType<LodGraphicsNode>().empty(),
          "the default room has no LOD objects (its plane and box are a handful of triangles)");
      chk(std::dynamic_pointer_cast<cvc::gl::GeometryNode>(sg.getGraphics("room_ground")) &&
              std::dynamic_pointer_cast<cvc::gl::GeometryNode>(sg.getGraphics("room_ref")),
          "both are plain geometry nodes");
      chk(rv.lod_builds() == 0 && rv.lod_stats().nodes == 0 && rv.lod_stats().changes == 0,
          "nothing is built and nothing selected");
    }
  }

  // ── the room's meshes are LOD ladders, chosen from the camera each frame ──
  {
    SceneGraph sg(app, "room_lod");
    // 800 px tall: the view the LOD presets' pixel budgets are stated for.
    SceneRenderer view(sg, 800, 800, /*offscreen=*/true, "main");
    holon h(holon_role::client);
    // A 40 m ground, the reference box and four bunnies (one asset, 69,473 triangles, fitted
    // 1.7 m tall), one per 20 m cell: bunny_<i>_<j> stands at (-10 + 20 i, -10 + 20 j).
    chk(rs::seed_large_demo_scene(app, "lod", 40.0, 2), "seeded a room with four bunnies");
    const char *const ids[4] = {"room_bunny_0_0", "room_bunny_1_0", "room_bunny_0_1",
                                "room_bunny_1_1"};
    // Near: the eye inside bunny_1_1's bounds, about 28 m from bunny_0_0. Far: a kilometre off.
    const double near_eye[3] = {10.0, 10.0, 0.85}, near_focal[3] = {0.0, 0.0, 0.85};
    const double far_eye[3] = {0.0, -1000.0, 300.0}, origin[3] = {0.0, 0.0, 0.0};
    const double dt = 1.0 / 60.0;
    {
      room_view rv(view, app, "lod", "A", sg.getStatePrefix());
      look(view, near_eye, near_focal);
      rv.update(h, dt);
      std::vector<std::shared_ptr<LodGraphicsNode>> lods;
      const auto fetch = [&] {
        lods.clear();
        for (const char *id : ids)
          if (std::shared_ptr<LodGraphicsNode> n = lod_node(sg, id))
            lods.push_back(n);
        return lods.size() == 4;
      };
      const auto all = [&](auto pred) {
        for (const std::shared_ptr<LodGraphicsNode> &n : lods)
          if (!pred(*n))
            return false;
        return !lods.empty();
      };
      const bool have = fetch();
      chk(have, "every bunny is a LOD object, under its room id");
      chk(sg.getGraphics("room_ref") && !lod_node(sg, "room_ref"), "the box stays a plain node");
      chk(all([](const LodGraphicsNode &n) { return n.rungCount() >= 1; }),
          "each draws its full-detail mesh at once");
      chk(rv.lod_builds() == 1, "the four share one build (one asset)");
      for (const std::string &w : rv.take_warnings())
        chk(false, "no realize or LOD warning (the bunny is under the native build cap): " + w);

      // The build runs on the cache's lane; update() hands its rungs to the bunnies when it lands.
      const auto t0 = std::chrono::steady_clock::now();
      const auto laddered = [&] {
        return all([](const LodGraphicsNode &n) { return n.rungCount() > 1; });
      };
      while (have && !laddered() &&
             std::chrono::steady_clock::now() - t0 < std::chrono::seconds(240)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        rv.update(h, dt);
      }
      printf(
          "  bunny pyramid landed after %.0f ms\n",
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
      chk(laddered(), "the build lands: every bunny gets the coarser rungs");
      chk(rv.lod_builds() == 1, "from that one build");

      if (have && laddered()) {
        const cvc::gl::lod_stats &st = rv.lod_stats(); // refreshed in place by every update
        const std::uint64_t full = lods[0]->rungTriangles(0);
        rv.update(h, dt);
        chk(lods[3]->selectedRung() == 0, "near: the bunny the eye stands in draws full detail");
        chk(lods[0]->selectedRung() > 0, "and the one 28 m away a coarser rung, the same frame");
        chk(st.nodes == 4 && st.full_tris == 4 * full && st.drawn_tris < st.full_tris &&
                st.drawn_tris >= full,
            "lod_stats: four nodes, fewer triangles drawn than at full detail");
        const std::uint64_t near_drawn = st.drawn_tris;

        look(view, far_eye, origin);
        rv.update(h, dt);
        chk(all([](const LodGraphicsNode &n) { return n.selectedRung() > 0; }) && st.changes >= 1,
            "far: the next update coarsens every bunny");
        chk(st.drawn_tris < near_drawn, "drawing fewer triangles than from near");

        // The preset is the selection's budget. At a distance where rung 1 of bunny_1_1 costs 1.8
        // px, aggressive (3.5 px) affords it and pristine (1 px) does not.
        const cvc::bounding_box bb = lods[3]->getWorldBoundingBox();
        const double c[3] = {0.5 * (bb.minx + bb.maxx), 0.5 * (bb.miny + bb.maxy),
                             0.5 * (bb.minz + bb.maxz)};
        const double r = 0.5 * std::sqrt((bb.maxx - bb.minx) * (bb.maxx - bb.minx) +
                                         (bb.maxy - bb.miny) * (bb.maxy - bb.miny) +
                                         (bb.maxz - bb.minz) * (bb.maxz - bb.minz));
        const double k = cvc::lod::k_px(cvc::gl::make_view_params(view.renderer()));
        const double d = lods[3]->rungError(1) * k / 1.8;
        const double mid_eye[3] = {c[0] + r + d, c[1], c[2]};
        look(view, mid_eye, c);
        rv.set_lod(true, cvc::lod::quality_preset::aggressive);
        rv.update(h, dt);
        const int aggressive = lods[3]->selectedRung();
        rv.set_lod(true, cvc::lod::quality_preset::pristine);
        rv.update(h, dt);
        const int pristine = lods[3]->selectedRung();
        chk(aggressive >= 1 && pristine == 0,
            "set_lod's preset is the budget: aggressive coarsens where pristine keeps full detail");
        rv.set_lod(true, cvc::lod::quality_preset::balanced);
        look(view, far_eye, origin);
        rv.update(h, dt);

        // Off: everything back to full detail, and kept there while the camera stays far.
        rv.set_lod(false, cvc::lod::quality_preset::balanced);
        rv.update(h, dt);
        const auto full_detail = [](const LodGraphicsNode &n) {
          return n.activeRung() == 0 && n.selectedRung() == 0;
        };
        chk(all(full_detail) && st.changes == 4 && st.drawn_tris == st.full_tris,
            "set_lod(false) pins every bunny to rung 0 (each a change, for the shadow re-bake)");
        rv.update(h, dt);
        chk(all(full_detail) && st.changes == 0, "and keeps them there, frame after frame");
        rv.set_lod(true, cvc::lod::quality_preset::balanced);
        rv.update(h, dt);
        chk(all([](const LodGraphicsNode &n) { return n.selectedRung() > 0; }),
            "set_lod(true) resumes the selection");

        // A revision bump re-realizes every object: new nodes, attached from the cache.
        const std::uint64_t rev0 = rv.realized_revision();
        const int rungs = lods[0]->rungCount();
        const std::weak_ptr<LodGraphicsNode> old = lods[0];
        lods.clear(); // the test's own references would keep the old nodes alive
        rs::remove_scene_object(app, "lod", "ref");
        rv.update(h, dt);
        chk(rv.realized_revision() != rev0 && sg.getGraphics("room_ref") == nullptr,
            "a revision bump re-realizes the room");
        const std::shared_ptr<LodGraphicsNode> before = old.lock(), again = lod_node(sg, ids[0]);
        chk(again && again != before, "as new LOD objects");
        chk(fetch() && all([rungs](const LodGraphicsNode &n) { return n.rungCount() == rungs; }),
            "each with the whole ladder at once, from the cache");
        chk(rv.lod_builds() == 1, "and no second build");
        chk(all([](const LodGraphicsNode &n) { return n.selectedRung() > 0; }) && st.nodes == 4,
            "selected in the same update that realized them");
      }
    }
    chk(sg.getAllGraphicsOfType<LodGraphicsNode>().empty(), "teardown removes the LOD objects");
  }

  printf("%s (%d failure%s)\n", fails ? "FAILED" : "OK", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
