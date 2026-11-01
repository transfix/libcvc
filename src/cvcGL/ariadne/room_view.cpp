// The conference room in 3D (H1 slice 6) — see inc/cvc/gl/ariadne/room_view.h.

#include <algorithm>
#include <cmath>
#include <cvc/ariadne/conference/room_avatar.h>
#include <cvc/ariadne/conference/room_intrinsics.h> // room_roster
#include <cvc/ariadne/conference/room_scene.h>
#include <cvc/ariadne/holon.h>
#include <cvc/ariadne/scene.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/LodGraphicsNode.h> // lod_stats, make_view_params
#include <cvc/gl/LodPyramidCache.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/room_view.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <set>
#include <utility>
#include <vtkRenderer.h>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace cvc {
namespace gl {
namespace ariadne {

namespace {

namespace rs = cvc::ariadne::conference;

// An avatar marker: a 0.5 x 0.5 x 1.7 m box standing on the pose (its base at the pose's z), the
// body every avatar has (room_avatar.h), with what it wears posed around it.
using rs::kAvatarHeight;
using rs::kAvatarWidth;
// Its visor: a dark slab across the front (+X in the avatar's frame) at head height, standing
// proud of the face so it reads from behind and the side too. Centred at kVisorOffset from the
// marker's centre.
constexpr float kVisorSize[3] = {0.14f, 0.40f, 0.16f};
constexpr double kVisorOffset[3] = {0.5 * kAvatarWidth, 0.0, 0.5 * kAvatarHeight - 0.32};
// A label floats this far above the pose (above the marker's top).
constexpr double kLabelHeight = kAvatarHeight + 0.35;

// The LOD error budget until set_lod says otherwise: in the browser, the wasm profile, which spends
// less of a smaller triangle and fill-rate budget.
#ifdef __EMSCRIPTEN__
constexpr cvc::lod::quality_preset kDefaultLodPreset = cvc::lod::quality_preset::aggressive;
#else
constexpr cvc::lod::quality_preset kDefaultLodPreset = cvc::lod::quality_preset::balanced;
#endif

std::string visor_node_id(const std::string &pid) {
  return room_view::avatar_node_id(pid) + "_visor";
}

std::string part_node_id(const std::string &pid, std::size_t i) {
  return room_view::avatar_node_id(pid) + "_part" + std::to_string(i);
}

// Frames between re-reads of the avatar catalog and everyone's choice (as for the names).
constexpr int kLooksEvery = 15;

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
                                             double yaw, double speed) {
  const double fx = std::cos(yaw), fy = std::sin(yaw);
  const double rx = fy, ry = -fx; // forward x up(0,0,1)
  const double f = (forward ? 1.0 : 0.0) - (back ? 1.0 : 0.0);
  const double r = (right ? 1.0 : 0.0) - (left ? 1.0 : 0.0);
  double mx = f * fx + r * rx, my = f * fy + r * ry;
  const double ml = std::hypot(mx, my);
  cvc::ariadne::avatar_intent in;
  in.yaw = yaw;
  if (ml > 1e-9) {
    mx *= speed / ml, my *= speed / ml;
    in.move = cvc::physics::vec3{mx, my, 0.0};
  }
  return in;
}

double heading_yaw(const double eye[3], const double focal[3]) {
  const double fx = focal[0] - eye[0], fy = focal[1] - eye[1];
  if (std::hypot(fx, fy) < 1e-9) // looking straight down (or up): no heading
    return 0.5 * 3.14159265358979323846;
  return std::atan2(fy, fx);
}

room_view::room_view(SceneRenderer &view, cvc::app &app, std::string room, std::string self_pid,
                     std::string bind_prefix)
    : _view(view), _app(app), _room(std::move(room)), _self(std::move(self_pid)),
      _prefix(std::move(bind_prefix)), _lod_cache(std::make_unique<LodPyramidCache>()),
      _lod_stats(std::make_unique<cvc::gl::lod_stats>()), _lod_preset(kDefaultLodPreset) {}

room_view::~room_view() {
  clear_scene();
  while (!_avatars.empty())
    remove_marker(_avatars.begin()->first);
  // _lod_cache goes with the members: it cancels its queued builds and waits for a running one.
}

std::vector<std::string> room_view::take_warnings() {
  std::vector<std::string> w;
  w.swap(_warnings);
  return w;
}

void room_view::set_lod(bool enabled, cvc::lod::quality_preset preset) {
  _lod_preset = preset;
  _view.scene().setLODEnabled(enabled);
}

const cvc::gl::lod_stats &room_view::lod_stats() const { return *_lod_stats; }

std::size_t room_view::lod_builds() const { return _lod_cache->builds(); }

void room_view::update(cvc::ariadne::holon &h, double wall_dt) {
  sync_scene();
  // Builds that landed since the last frame reach their objects before this frame's selection.
  // Where builds have no thread this also runs one queued build, here, on the render thread.
  _lod_cache->poll();
  // A mesh over the build cap (the bunny on single-threaded wasm) or a failed build says so here.
  for (std::string &w : _lod_cache->take_warnings())
    _warnings.push_back(std::move(w));
  sync_avatars(h, wall_dt);
  sync_names();
  select_lod();
}

void room_view::select_lod() {
  vtkRenderer *r = _view.renderer();
  // Nothing to select without LOD objects. Nor before the renderer has a camera: make_view_params
  // never creates one, so it would select from the preset's default eye (the origin). The ladders
  // draw rung 0 until a selection is made.
  if (!_realized || _realized->lod_nodes.empty() || !r || !r->IsActiveCameraCreated()) {
    _lod_stats->reset();
    return;
  }
  SceneGraph &sg = _view.scene();
  const cvc::lod::view_params view = make_view_params(r, cvc::lod::preset_view(_lod_preset));
  // A rung switch is geometry appearing and disappearing: a shadow map baked every Nth frame must
  // be re-baked, or the old rung goes on casting.
  if (sg.selectLOD(view, _lod_stats.get()) > 0)
    sg.invalidateShadowBake();
}

void room_view::sync_names() {
  // Names change rarely (a join, a rename): re-read the roster a few times a second, not per frame.
  if (_names_age-- > 0)
    return;
  _names_age = 15;
  _names.clear();
  for (const rs::room_participant &p : rs::room_roster(_app, _room))
    _names[p.pid] = p.name;
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
  // Mesh objects become LOD ladders from the long-lived cache: a revision change re-realizes
  // every object, and the cache turns that into a re-attach rather than a rebuild.
  RealizeOptions options;
  options.lod = _lod_cache.get();
  _realized =
      std::make_unique<RealizedScene>(realize_scene(sg, scene, _prefix, &_warnings, options));
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
  // The participants to draw: one whose pose is not known yet (registered from the roster before
  // its first sample) has nowhere to stand but the placeholder origin, so it gets no marker.
  std::vector<std::string> pids;
  for (const std::string &pid : h.participants())
    if (h.pose_known(pid))
      pids.push_back(pid);
  const std::set<std::string> present(pids.begin(), pids.end());

  // Departed participants (or one re-registered pose-less under the same pid): drop their markers.
  std::vector<std::string> gone;
  for (const auto &kv : _avatars)
    if (!present.count(kv.first))
      gone.push_back(kv.first);
  for (const std::string &pid : gone) {
    remove_marker(pid);
    _heads.erase(pid);
  }
  if (!present.count(_self))
    _have_self = false;

  sync_looks(pids);
  for (const std::string &pid : pids) {
    const bool self = (pid == _self);
    const std::string id = avatar_node_id(pid);
    const rs::avatar_look &look = _looks[pid];
    const bool known = _avatars.count(pid) != 0;
    if (!known || _avatars[pid].key != look.key)
      build_marker(pid, look); // new, or it chose something else (or the catalog changed)
    if (!known) {
      if (self) {
        // Its first pose is where it starts, not a correction to ease out: drop the jump from the
        // placeholder it was predicted at before the first sample, so the marker appears there
        // instead of gliding in from the origin.
        h.take_correction(pid);
        _self_offset = cvc::physics::vec3{0.0, 0.0, 0.0};
      }
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
    double m[16];
    // The box is centred on its origin: lift it by half its height to stand on the pose.
    pose_matrix(p.rotation, p.position.x, p.position.y, p.position.z + 0.5 * kAvatarHeight, m);
    if (std::shared_ptr<GraphicsNode> node = sg.getGraphics(id))
      node->setPoseMatrix(m);
    // The visor and what it wears: the same rotation, each centre moved by its offset in the
    // avatar's frame (R * offset) -- the visor's from the box's centre, a part's from the feet.
    const auto place = [&](const std::string &node_id, const double off[3]) {
      if (std::shared_ptr<GraphicsNode> g = sg.getGraphics(node_id)) {
        double pm[16];
        std::copy(m, m + 16, pm);
        for (int r = 0; r < 3; ++r)
          pm[4 * r + 3] += m[4 * r] * off[0] + m[4 * r + 1] * off[1] + m[4 * r + 2] * off[2];
        g->setPoseMatrix(pm);
      }
    };
    place(visor_node_id(pid), kVisorOffset);
    const std::vector<rs::avatar_part> &parts = _avatars[pid].parts;
    for (std::size_t i = 0; i < parts.size(); ++i) {
      const double off[3] = {parts[i].pos[0], parts[i].pos[1],
                             parts[i].pos[2] - 0.5 * kAvatarHeight};
      place(part_node_id(pid, i), off);
    }
    if (self) { // shown or hidden (set_self_visible); a rebuilt marker starts shown
      const auto show = [&](const std::string &node_id) {
        if (std::shared_ptr<GraphicsNode> g = sg.getGraphics(node_id);
            g && g->isVisible() != _self_visible)
          g->setVisible(_self_visible);
      };
      show(id);
      show(visor_node_id(pid));
      for (std::size_t i = 0; i < parts.size(); ++i)
        show(part_node_id(pid, i));
    }
    _heads[pid] = cvc::physics::vec3{p.position.x, p.position.y, p.position.z + kLabelHeight};
  }
}

void room_view::sync_looks(const std::vector<std::string> &pids) {
  const bool refresh = _looks_age-- <= 0;
  if (refresh) {
    _looks_age = kLooksEvery;
    const std::uint64_t rev = rs::avatar_catalog_revision(_app, _room);
    if (rev != _catalog_rev) {
      _catalog = rs::read_avatar_catalog(_app, _room);
      _catalog_rev = rev;
    }
  }
  std::map<std::string, rs::avatar_look> looks;
  for (const std::string &pid : pids) {
    const auto it = _looks.find(pid);
    if (!refresh && it != _looks.end()) {
      looks.emplace(pid, std::move(it->second));
      continue;
    }
    // A newcomer is resolved at once, so its marker is built as it chose rather than rebuilt.
    looks.emplace(pid,
                  rs::resolve_avatar_look(_catalog, rs::read_avatar_choice(_app, _room, pid), pid));
  }
  _looks.swap(looks);
}

void room_view::build_marker(const std::string &pid, const rs::avatar_look &look) {
  remove_marker(pid);
  cvc::ariadne::Scene one;
  cvc::ariadne::SceneNode n;
  n.id = avatar_node_id(pid);
  n.source_primitive = "box";
  n.box_size[0] = n.box_size[1] = kAvatarWidth;
  n.box_size[2] = kAvatarHeight;
  n.has_material = true;
  std::copy(look.body, look.body + 3, n.color);
  n.ambient = 0.3f;
  n.diffuse = 0.8f;
  one.nodes.push_back(n);
  cvc::ariadne::SceneNode v = n; // the visor: posed with the marker every frame
  v.id = visor_node_id(pid);
  v.box_size[0] = kVisorSize[0], v.box_size[1] = kVisorSize[1], v.box_size[2] = kVisorSize[2];
  std::copy(look.visor, look.visor + 3, v.color);
  one.nodes.push_back(v);
  for (std::size_t i = 0; i < look.parts.size(); ++i) {
    const rs::avatar_part &part = look.parts[i];
    cvc::ariadne::SceneNode p = n;
    p.id = part_node_id(pid, i);
    p.source_primitive = part.shape;
    if (part.shape == "sphere")
      p.sphere_radius = part.radius;
    else
      std::copy(part.size, part.size + 3, p.box_size);
    std::copy(part.color, part.color + 3, p.color);
    one.nodes.push_back(p);
  }
  realize_scene(_view.scene(), one, _prefix, &_warnings); // plain geometry nodes: nothing to keep
  _avatars[pid] = look;
}

void room_view::remove_marker(const std::string &pid) {
  const auto it = _avatars.find(pid);
  if (it == _avatars.end())
    return;
  SceneGraph &sg = _view.scene();
  sg.removeGraphics(avatar_node_id(pid));
  sg.removeGraphics(visor_node_id(pid));
  for (std::size_t i = 0; i < it->second.parts.size(); ++i)
    sg.removeGraphics(part_node_id(pid, i));
  _avatars.erase(it);
}

bool room_view::avatar_look(const std::string &pid, rs::avatar_look &out) const {
  const auto it = _avatars.find(pid);
  if (it == _avatars.end())
    return false;
  out = it->second;
  return true;
}

std::vector<std::string> room_view::avatar_pids() const {
  std::vector<std::string> out;
  for (const auto &kv : _avatars)
    out.push_back(kv.first);
  return out;
}

bool room_view::self_position(double out[3]) const {
  if (!_have_self)
    return false;
  out[0] = _self_pos[0], out[1] = _self_pos[1], out[2] = _self_pos[2];
  return true;
}

bool room_view::pick_ground(int x, int y, double ground_z, double out[3]) const {
  vtkRenderer *r = _view.renderer();
  if (!r)
    return false;
  // The pixel's ray: its points on the near (depth 0) and far (depth 1) planes, in world space.
  double ends[2][3];
  for (int i = 0; i < 2; ++i) {
    r->SetDisplayPoint(static_cast<double>(x), static_cast<double>(y), static_cast<double>(i));
    r->DisplayToWorld();
    const double *w = r->GetWorldPoint();
    if (std::fabs(w[3]) < 1e-12)
      return false;
    ends[i][0] = w[0] / w[3], ends[i][1] = w[1] / w[3], ends[i][2] = w[2] / w[3];
  }
  const double dz = ends[1][2] - ends[0][2];
  if (std::fabs(dz) < 1e-12) // parallel to the ground
    return false;
  const double t = (ground_z - ends[0][2]) / dz;
  if (t < 0.0) // the plane is behind the camera
    return false;
  for (int k = 0; k < 3; ++k)
    out[k] = ends[0][k] + t * (ends[1][k] - ends[0][k]);
  out[2] = ground_z;
  return true;
}

void room_view::draw_labels() const {
#ifdef CVC_ENABLE_IMGUI
  vtkRenderer *r = _view.renderer();
  if (!r || !ImGui::GetCurrentContext())
    return;
  const float h = ImGui::GetIO().DisplaySize.y;    // the window's height in display pixels
  ImDrawList *dl = ImGui::GetBackgroundDrawList(); // under the UI's windows
  for (const auto &kv : _heads) {
    if (kv.first == _self) // you are the gold one; your own name would only cover the view
      continue;
    const cvc::physics::vec3 &w = kv.second;
    r->SetWorldPoint(w.x, w.y, w.z, 1.0);
    r->WorldToDisplay();
    const double *d = r->GetDisplayPoint();
    if (d[2] < 0.0 || d[2] > 1.0) // behind the camera (or past the far plane)
      continue;
    auto nm = _names.find(kv.first);
    const std::string &text = (nm == _names.end() || nm->second.empty()) ? kv.first : nm->second;
    const ImVec2 sz = ImGui::CalcTextSize(text.c_str());
    // VTK's display origin is bottom-left, ImGui's top-left.
    const ImVec2 at(static_cast<float>(d[0]) - 0.5f * sz.x, h - static_cast<float>(d[1]) - sz.y);
    dl->AddText(ImVec2(at.x + 1.0f, at.y + 1.0f), IM_COL32(0, 0, 0, 200), text.c_str());
    dl->AddText(at, IM_COL32(255, 255, 255, 235), text.c_str());
  }
#endif
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
