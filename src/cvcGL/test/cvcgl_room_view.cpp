// H1 slice 6: cvc::gl::ariadne::room_view — the conference room in 3D, offscreen. The room scene
// from the replicated tree is realized (and re-realized on a revision change), avatar markers
// follow a client holon's participants and poses, a local reconcile jump is eased (no visible
// step), the key -> intent mapping walks and strafes relative to the avatar's facing, the visor
// turns with the facing, a pixel picks the ground point under it, and teardown removes every node
// room_view made.
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/avatar_sim.h>
#include <cvc/ariadne/conference/room_scene.h>
#include <cvc/ariadne/holon.h>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/room_view.h>
#include <string>
#include <vtkCamera.h>
#include <vtkMatrix4x4.h>
#include <vtkRenderer.h>

using cvc::ariadne::avatar_intent;
using cvc::ariadne::holon;
using cvc::ariadne::holon_role;
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

  printf("%s (%d failure%s)\n", fails ? "FAILED" : "OK", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
