// Keyframe animation (docs/ARIADNE.md §5.4, "Animations"): a scene's `animations:` played by
// realize_scene and tick_scene with no window, the scene clock stepped a fixed 0.05 s a frame
// (under the clock's cap of 8 quanta of 1/120 s a frame, so each frame is all of it).
//
// Pins:
//   - the start pose is set at realize, playing or not, and the keys say where each animation is;
//   - a once animation moves its node along its keyframes (linear rotation, a step position) and
//     stops at its end;
//   - a loop wraps: a mesh's colour and a state key follow it;
//   - a paused pingpong holds until its playing key is written, then bounces; a node id moves every
//     node with it (each repeat: copy's child), a path one alone;
//   - a script seeks (time), pauses and plays (playing), and changes the speed (speed); a once
//     animation played again after its end starts over;
//   - a paused scene clock holds every animation.
#include "streaming_test_util.h"

#include <cmath>
#include <cstdio>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/state/state.h>
#include <memory>
#include <string>
#include <vector>
#include <vtkActor.h>
#include <vtkMatrix4x4.h>
#include <vtkProperty.h>

namespace {

namespace gla = cvc::gl::ariadne;
using cvc::gl::GraphicsNode;
using cvcgl_test::check;

const char *kDoc = R"(meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - { node: door, source: { box: [1, 0.1, 2] } }
    - { node: lamp, source: { sphere: 0.3 }, material: { color: [1, 1, 1] } }
    - node: "post_{i}"
      repeat: 2
      source: { box: 0.2 }
      transform: { position: [0, 3, 0] }
      children:
        - { node: flag, source: { box: 0.1 } }
  animations:
    - animation: swing
      tracks:
        - { node: door, key: rotation, keys: [[0, [0, 0, 0]], [2, [0, 0, 90]]] }
        - { node: door, key: position, interpolation: step, keys: [[0, [0, 0, 0]], [1, [5, 0, 0]]] }
    - animation: glow
      loop: true
      tracks:
        - { node: lamp, key: color, keys: [[0, [1, 0, 0]], [1, [0, 0, 1]]] }
        - { state: lamp.level, keys: [[0, 0], [1, 10]] }
    - animation: bob
      loop: pingpong
      autoplay: false
      tracks:
        - { node: flag, key: position, keys: [[0, [0, 0, 0]], [1, [0, 0, 1]]] }
        - { node: post_1/flag, key: scale, keys: [[0, [1, 1, 1]], [1, [2, 2, 2]]] }
)";

cvc::state &key(cvc::app &app, const std::string &path) { return cvc::state::instance(app)(path); }
std::string text(cvc::app &app, const std::string &path) { return key(app, path).value(); }
double num(cvc::app &app, const std::string &path) {
  try {
    return std::stod(text(app, path));
  } catch (...) {
    return std::nan("");
  }
}

// A node's local pose: its origin's offset, and the angle (degrees) its x axis makes about z.
void pose(const std::shared_ptr<GraphicsNode> &n, double p[3], double &yaw) {
  p[0] = p[1] = p[2] = std::nan("");
  yaw = std::nan("");
  if (!n)
    return;
  const double o[3] = {0, 0, 0}, x[3] = {1, 0, 0};
  double w0[3], w1[3];
  n->localToWorld(o, w0);
  n->localToWorld(x, w1);
  for (int k = 0; k < 3; ++k)
    p[k] = w0[k];
  yaw = std::atan2(w1[1] - w0[1], w1[0] - w0[0]) * 180.0 / 3.14159265358979;
}

double length_of_x(const std::shared_ptr<GraphicsNode> &n) { // its scale along x, in the world
  if (!n)
    return std::nan("");
  const double o[3] = {0, 0, 0}, x[3] = {1, 0, 0};
  double w0[3], w1[3];
  n->localToWorld(o, w0);
  n->localToWorld(x, w1);
  return std::sqrt((w1[0] - w0[0]) * (w1[0] - w0[0]) + (w1[1] - w0[1]) * (w1[1] - w0[1]) +
                   (w1[2] - w0[2]) * (w1[2] - w0[2]));
}

double green_of(const std::shared_ptr<GraphicsNode> &n, int channel) {
  auto *actor = n ? vtkActor::SafeDownCast(n->prop()) : nullptr;
  return actor ? actor->GetProperty()->GetColor()[channel] : std::nan("");
}

bool near(double a, double b, double tol = 1e-6) { return std::fabs(a - b) < tol; }

} // namespace

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  const cvc::ariadne::LoadResult lr = cvc::ariadne::load_string(app, kDoc);
  check(lr.ok, "the document loads", lr.error);
  for (const std::string &w : lr.warnings)
    check(false, "the document loads without warnings", w);
  if (!lr.ok)
    return cvcgl_test::finish("cvcgl_ariadne_animation");

  cvc::gl::SceneGraph sg(app, "anim");
  std::vector<std::string> warns;
  gla::RealizedScene rs = gla::realize_scene(sg, lr.scene, "anim", &warns);
  for (const std::string &w : warns)
    check(false, "it realizes without warnings", w);
  const auto tick = [&](int frames) {
    for (int f = 0; f < frames; ++f)
      gla::tick_scene(rs, nullptr, 0.05);
    sg.processEvents();
  };
  const std::shared_ptr<GraphicsNode> door = sg.getGraphics("door"), lamp = sg.getGraphics("lamp");
  const std::shared_ptr<GraphicsNode> post0 = sg.getGraphics("post_0"),
                                      post1 = sg.getGraphics("post_1");
  const std::shared_ptr<GraphicsNode> flag0 = post0 ? post0->findChildByName("flag") : nullptr,
                                      flag1 = post1 ? post1->findChildByName("flag") : nullptr;
  check(door && lamp && flag0 && flag1, "the nodes exist (and both copies' flags)");
  double p[3], yaw;

  std::printf("At realize\n");
  check(rs.clock.present, "an animated scene runs on the scene clock");
  pose(door, p, yaw);
  check(near(yaw, 0.0) && near(p[0], 0.0), "the door at its first keyframe");
  check(near(green_of(lamp, 0), 1.0) && near(green_of(lamp, 2), 0.0), "the lamp red");
  check(text(app, "anim.animations.swing.time") == "0" &&
            text(app, "anim.animations.swing.playing") == "1" &&
            text(app, "anim.animations.bob.playing") == "0" &&
            text(app, "anim.animations.glow.speed") == "1",
        "the keys: time 0, swing playing, bob paused, speed 1");

  std::printf("Half a second\n");
  tick(10);
  pose(door, p, yaw);
  check(near(yaw, 22.5, 1e-4), "the door turned a quarter of the way", std::to_string(yaw));
  check(near(p[0], 0.0), "its position still the first key (step)");
  check(near(green_of(lamp, 0), 0.5, 1e-4) && near(green_of(lamp, 2), 0.5, 1e-4),
        "the lamp halfway from red to blue");
  check(near(num(app, "anim.lamp.level"), 5.0, 1e-4), "the state track: lamp.level is 5",
        text(app, "anim.lamp.level"));
  check(near(num(app, "anim.animations.swing.time"), 0.5, 1e-6), "swing's time is 0.5");
  pose(flag0, p, yaw);
  check(near(p[2], 0.0), "bob, paused, holds");

  std::printf("Past the end of a once animation\n");
  tick(40); // 2.5 s
  pose(door, p, yaw);
  check(near(yaw, 90.0, 1e-4) && near(p[0], 5.0), "the door at its last keyframe");
  check(text(app, "anim.animations.swing.playing") == "0" &&
            near(num(app, "anim.animations.swing.time"), 2.0),
        "swing stopped at its end");
  check(near(green_of(lamp, 0), 0.5, 1e-4), "glow looped: half a second into its third pass");

  std::printf("A script plays bob\n");
  key(app, "anim.animations.bob.playing").value(std::string("1"));
  tick(30); // 1.5 s: there and half way back
  pose(flag0, p, yaw);
  const double z0 = p[2] - 0.0;
  pose(flag1, p, yaw);
  check(near(z0, 0.5, 1e-4) && near(p[2], 0.5, 1e-4), "every post's flag bounced back to 0.5",
        std::to_string(z0) + ", " + std::to_string(p[2]));
  check(near(length_of_x(flag1), 1.5, 1e-4) && near(length_of_x(flag0), 1.0, 1e-6),
        "post_1/flag alone scaled");

  std::printf("Seek, pause, play, speed\n");
  key(app, "anim.animations.swing.time").value(std::string("1"));
  tick(1); // not playing: it stays where it was put
  pose(door, p, yaw);
  check(near(yaw, 45.0, 1e-4), "a seek to 1 s turns the door back to 45 degrees",
        std::to_string(yaw));
  key(app, "anim.animations.swing.playing").value(std::string("true"));
  tick(10);
  check(near(num(app, "anim.animations.swing.time"), 1.5, 1e-6), "and playing goes on from there");
  tick(20);
  key(app, "anim.animations.swing.playing").value(std::string("1"));
  tick(1);
  check(near(num(app, "anim.animations.swing.time"), 0.05, 1e-6),
        "played again after its end, it starts over", text(app, "anim.animations.swing.time"));
  key(app, "anim.animations.glow.playing").value(std::string("0"));
  const double held = num(app, "anim.animations.glow.time");
  tick(3);
  check(near(num(app, "anim.animations.glow.time"), held), "paused, glow holds");
  key(app, "anim.animations.glow.playing").value(std::string("1"));
  key(app, "anim.animations.glow.speed").value(std::string("2"));
  key(app, "anim.animations.glow.time").value(std::string("0"));
  tick(4);
  check(near(num(app, "anim.animations.glow.time"), 0.4, 1e-6), "at speed 2, 0.2 s is 0.4",
        text(app, "anim.animations.glow.time"));

  std::printf("A paused clock\n");
  key(app, "anim.sim.paused").value(std::string("1"));
  const double before = num(app, "anim.animations.glow.time");
  tick(5);
  check(near(num(app, "anim.animations.glow.time"), before), "the scene clock paused: glow holds");

  return cvcgl_test::finish("cvcgl_ariadne_animation");
}
