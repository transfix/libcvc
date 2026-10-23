// The conference room in 3D (H1 slice 6) — see inc/cvc/gl/ariadne/room_view.h.

#include <algorithm>
#include <cmath>
#include <cvc/ariadne/conference/room_scene.h>
#include <cvc/ariadne/holon.h>
#include <cvc/ariadne/scene.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/room_view.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <set>
#include <utility>
#include <vtkRenderer.h>

namespace cvc {
namespace gl {
namespace ariadne {

namespace {

namespace rs = cvc::ariadne::conference;

// An avatar marker: a 0.5 x 0.5 x 1.7 m box standing on the pose (its base at the pose's z).
constexpr float kAvatarWidth = 0.5f;
constexpr float kAvatarHeight = 1.7f;

// A stable, distinct colour per peer (a hue from the pid's hash); the local avatar is gold.
void avatar_color(const std::string &pid, bool self, float out[3]) {
  if (self) {
    out[0] = 0.95f, out[1] = 0.75f, out[2] = 0.20f;
    return;
  }
  std::uint32_t h = 2166136261u; // FNV-1a: the same pid colours the same on every client
  for (unsigned char c : pid)
    h = (h ^ c) * 16777619u;
  const float hue = static_cast<float>(h % 360u) / 60.0f; // [0, 6)
  const float s = 0.55f, v = 0.85f, c = v * s;
  const float x = c * (1.0f - std::fabs(std::fmod(hue, 2.0f) - 1.0f));
  float r = 0, g = 0, b = 0;
  switch (static_cast<int>(hue)) {
  case 0:
    r = c, g = x;
    break;
  case 1:
    r = x, g = c;
    break;
  case 2:
    g = c, b = x;
    break;
  case 3:
    g = x, b = c;
    break;
  case 4:
    r = x, b = c;
    break;
  default:
    r = c, b = x;
    break;
  }
  out[0] = r + (v - c), out[1] = g + (v - c), out[2] = b + (v - c);
}

// Row-major 4x4 of a rotation quaternion and a translation.
void pose_matrix(const cvc::physics::quat &q, double tx, double ty, double tz, double m[16]) {
  const double w = q.w, x = q.x, y = q.y, z = q.z;
  m[0] = 1 - 2 * (y * y + z * z), m[1] = 2 * (x * y - w * z), m[2] = 2 * (x * z + w * y), m[3] = tx;
  m[4] = 2 * (x * y + w * z), m[5] = 1 - 2 * (x * x + z * z), m[6] = 2 * (y * z - w * x), m[7] = ty;
  m[8] = 2 * (x * z - w * y), m[9] = 2 * (y * z + w * x), m[10] = 1 - 2 * (x * x + y * y),
  m[11] = tz;
  m[12] = 0, m[13] = 0, m[14] = 0, m[15] = 1;
}

} // namespace

cvc::ariadne::avatar_intent intent_from_keys(bool forward, bool back, bool left, bool right,
                                             const double eye[3], const double focal[3],
                                             double speed) {
  double fx = focal[0] - eye[0], fy = focal[1] - eye[1];
  const double fl = std::hypot(fx, fy);
  if (fl < 1e-9) { // looking straight down (or up): no heading
    fx = 0.0, fy = 1.0;
  } else {
    fx /= fl, fy /= fl;
  }
  const double rx = fy, ry = -fx; // forward x up(0,0,1)
  const double f = (forward ? 1.0 : 0.0) - (back ? 1.0 : 0.0);
  const double r = (right ? 1.0 : 0.0) - (left ? 1.0 : 0.0);
  double mx = f * fx + r * rx, my = f * fy + r * ry;
  const double ml = std::hypot(mx, my);
  cvc::ariadne::avatar_intent in;
  if (ml > 1e-9) {
    mx *= speed / ml, my *= speed / ml;
    in.move = cvc::physics::vec3{mx, my, 0.0};
  }
  return in;
}

room_view::room_view(SceneRenderer &view, cvc::app &app, std::string room, std::string self_pid,
                     std::string bind_prefix)
    : _view(view), _app(app), _room(std::move(room)), _self(std::move(self_pid)),
      _prefix(std::move(bind_prefix)) {}

room_view::~room_view() {
  clear_scene();
  SceneGraph &sg = _view.scene();
  for (const auto &kv : _avatars)
    sg.removeGraphics(avatar_node_id(kv.first));
}

std::vector<std::string> room_view::take_warnings() {
  std::vector<std::string> w;
  w.swap(_warnings);
  return w;
}

void room_view::update(cvc::ariadne::holon &h, double wall_dt) {
  sync_scene();
  sync_avatars(h, wall_dt);
}

void room_view::clear_scene() {
  SceneGraph &sg = _view.scene();
  for (const std::string &id : _scene_ids)
    sg.removeGraphics(id);
  _scene_ids.clear();
  _realized.reset(); // after the nodes: ~StageLighting removes a rig's own lights
}

void room_view::sync_scene() {
  const std::uint64_t rev = rs::scene_revision(_app, _room);
  if (rev == _realized_rev)
    return;
  clear_scene();
  cvc::ariadne::Scene scene = rs::read_room_scene(_app, _room);
  for (cvc::ariadne::SceneNode &n : scene.nodes)
    n.id = object_node_id(n.id);
  for (cvc::ariadne::SceneLight &l : scene.lights)
    l.id = object_node_id(l.id);
  SceneGraph &sg = _view.scene();
  _realized = std::make_unique<RealizedScene>(realize_scene(sg, scene, _prefix, &_warnings));
  _scene_ids = _realized->created;
  for (const cvc::ariadne::SceneLight &l : scene.lights)
    if (l.rig.empty()) // a rig's lights belong to (and leave with) its StageLighting
      _scene_ids.push_back(l.id);
  // The background is a view property (as AriRuntime applies a document's).
  if (scene.has_background) {
    const float *t = scene.background_top, *b = scene.background_bottom;
    vtkRenderer *r = _view.renderer();
    if (scene.background_gradient && r) {
      r->SetGradientBackground(true);
      r->SetBackground(b[0], b[1], b[2]);  // bottom
      r->SetBackground2(t[0], t[1], t[2]); // top
    } else {
      if (r)
        r->SetGradientBackground(false);
      _view.setBackground(t[0], t[1], t[2]);
    }
  }
  _realized_rev = rev;
}

void room_view::sync_avatars(cvc::ariadne::holon &h, double wall_dt) {
  SceneGraph &sg = _view.scene();
  const std::vector<std::string> pids = h.participants();
  const std::set<std::string> present(pids.begin(), pids.end());

  // Departed participants: drop their markers.
  for (auto it = _avatars.begin(); it != _avatars.end();) {
    if (!present.count(it->first)) {
      sg.removeGraphics(avatar_node_id(it->first));
      it = _avatars.erase(it);
    } else {
      ++it;
    }
  }
  if (!present.count(_self))
    _have_self = false;

  for (const std::string &pid : pids) {
    const bool self = (pid == _self);
    const std::string id = avatar_node_id(pid);
    if (!_avatars.count(pid)) {
      cvc::ariadne::Scene one;
      cvc::ariadne::SceneNode n;
      n.id = id;
      n.source_primitive = "box";
      n.box_size[0] = n.box_size[1] = kAvatarWidth;
      n.box_size[2] = kAvatarHeight;
      n.has_material = true;
      avatar_color(pid, self, n.color);
      n.ambient = 0.3f;
      n.diffuse = 0.8f;
      one.nodes.push_back(n);
      realize_scene(sg, one, _prefix, &_warnings); // a plain geometry node: nothing to keep
      _avatars[pid] = true;
    }

    cvc::physics::pose p = h.display_pose(pid);
    if (self) {
      // Ease the local avatar's reconcile corrections: absorb each jump into an offset
      // (so the shown position does not move this frame), then decay the offset.
      const cvc::physics::vec3 c = h.take_correction(pid);
      _self_offset = _self_offset - c;
      const double k = wall_dt > 0.0 ? std::exp(-wall_dt / std::max(correction_tau, 1e-6)) : 1.0;
      _self_offset = cvc::physics::vec3{_self_offset.x * k, _self_offset.y * k, _self_offset.z * k};
      p.position = p.position + _self_offset;
      _self_pos[0] = p.position.x, _self_pos[1] = p.position.y, _self_pos[2] = p.position.z;
      _have_self = true;
    }
    if (std::shared_ptr<GraphicsNode> node = sg.getGraphics(id)) {
      double m[16];
      // The box is centred on its origin: lift it by half its height to stand on the pose.
      pose_matrix(p.rotation, p.position.x, p.position.y, p.position.z + 0.5 * kAvatarHeight, m);
      node->setPoseMatrix(m);
    }
  }
}

bool room_view::self_position(double out[3]) const {
  if (!_have_self)
    return false;
  out[0] = _self_pos[0], out[1] = _self_pos[1], out[2] = _self_pos[2];
  return true;
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
