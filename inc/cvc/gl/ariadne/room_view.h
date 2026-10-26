#ifndef CVC_GL_ARIADNE_ROOM_VIEW_H
#define CVC_GL_ARIADNE_ROOM_VIEW_H

// The conference room in 3D (ARIADNE_CONFERENCE_ROADMAP.md §5g, H1 slice 6): the client side of
// the server-authoritative shared scene and the avatars.
//
//   * The room scene (conf.rooms.<room>.scene.*, cvc::ariadne::conference::room_scene) is realized
//     into the viewer's SceneGraph with realize_scene, and re-realized whenever its revision
//     changes. Its node and light ids are prefixed "room_" so they never collide with the hosting
//     document's own scene.
//   * One marker per participant of a CLIENT holon (cvc::ariadne::client_pump keeps the holon in
//     step with the replicated room), posed every frame at holon::display_pose: predicted for the
//     local avatar, a smooth walk between samples for a peer. A reconcile jump on the local avatar
//     (holon::take_correction) is eased out over `correction_tau`, so a server correction reads
//     as a glide rather than a jump. A participant whose pose is not known yet
//     (holon::pose_known: in the roster, no sample so far) has no marker and no label, rather
//     than one standing at the origin; the local avatar's marker appears at its first pose, not
//     gliding in from the origin it was predicted at before that.
//   * Each marker carries a dark visor on its front face, so its facing (the pose rotation, from
//     the participant's intent.yaw) reads at a glance; each peer's display name floats above it
//     (draw_labels, from the host's ImGuiOverlay scene-draw callback).
//   * Third-person controls: intent_from_keys turns held WASD / arrow keys into forward / back /
//     strafe relative to the avatar's FACING; the host turns the avatar by a click on the scene
//     (pick_ground -> face that point). self_position is where the host keeps the orbit centre.
//   * The room's mesh objects (a URI source with enough triangles to gain from it; see
//     RealizeOptions) are realized as LOD ladders: full detail at once, the coarser rungs once a
//     background build lands, and each frame one rung per object, chosen from the view's camera
//     (SceneGraph::selectLOD). Their pyramids are kept for the room_view's whole life, so a
//     re-realize on a revision change attaches them again without rebuilding, and one asset shown
//     by many objects is built once. The default room's plane and box are a handful of triangles:
//     there, LOD does nothing.
//
// Owner/render thread, like the SceneGraph it draws into.

#include <cstddef>
#include <cstdint>
#include <cvc/ariadne/avatar_sim.h> // avatar_intent
#include <cvc/lod/select.h>         // quality_preset
#include <cvc/physics/types.h>      // vec3
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace ariadne {
class holon;
} // namespace ariadne
namespace gl {
class SceneRenderer;
class LodPyramidCache;
struct lod_stats;
namespace ariadne {
struct RealizedScene;

// The local avatar's intent from the held movement keys, relative to its facing YAW (radians about
// +Z from +X, Z-up): W/S walk forward/back along the facing, A/D strafe left/right of it (right is
// forward x up). Diagonals are normalized (never faster than `speed`); opposing keys cancel. The
// intent carries `yaw`, so the avatar keeps facing it while it walks or strafes.
cvc::ariadne::avatar_intent intent_from_keys(bool forward, bool back, bool left, bool right,
                                             double yaw, double speed);

// The camera's heading on a Z-up ground as a yaw (radians about +Z from +X): the view direction
// (eye -> focal) flattened onto the ground. A camera looking straight down has none: +Y (pi/2).
double heading_yaw(const double eye[3], const double focal[3]);

class room_view {
public:
  // Draws ROOM's scene and SELF_PID's view of the avatars into `view`'s SceneGraph. BIND_PREFIX is
  // the state prefix realize_scene resolves binds against (the host's, e.g. sg.getStatePrefix()).
  room_view(SceneRenderer &view, cvc::app &app, std::string room, std::string self_pid,
            std::string bind_prefix);
  // Removes every node it created, then waits for the LOD build still running, if any (queued
  // ones are cancelled).
  ~room_view();
  room_view(const room_view &) = delete;
  room_view &operator=(const room_view &) = delete;

  // Once per frame, after client_pump advanced the holon: re-realize the room scene if its
  // revision changed, hand finished LOD builds to the objects waiting for them, add/remove avatar
  // markers to match the holon's participants and pose them, then choose each LOD object's rung
  // from the camera as it stands now (so call it before rendering the frame). wall_dt (seconds)
  // paces the correction easing.
  void update(cvc::ariadne::holon &h, double wall_dt);

  // Per-frame LOD for the room's mesh objects. ENABLED is the view SceneGraph's own switch
  // (SceneGraph::setLODEnabled), so it covers every LOD node of that scene: off pins each one to
  // rung 0, full detail -- the A/B toggle. update() runs the selection pass either way; while off,
  // the pass is what holds the pin. PRESET is the screen-error budget (cvc::lod::preset_view).
  // Until this is called the switch is as the host left it (on, by default) and the preset is
  // balanced natively, aggressive under Emscripten (the wasm profile).
  void set_lod(bool enabled, cvc::lod::quality_preset preset);

  // What the last update()'s LOD selection did (SceneGraph::selectLOD's stats: the rung histogram,
  // the rung changes, drawn vs full-detail triangles). All zero when that update had no LOD object
  // to select, or no camera to select from yet.
  const cvc::gl::lod_stats &lod_stats() const;

  // LOD pyramid builds started over this view's life: one per distinct mesh, however many objects
  // show it and however often the scene is re-realized.
  std::size_t lod_builds() const;

  // Where the local avatar is shown (the eased display position, at the marker's feet). False
  // until the holon has the local avatar with a known pose.
  bool self_position(double out[3]) const;

  // The point on the horizontal plane z = GROUND_Z under display pixel (X, Y) (VTK display
  // coordinates, origin bottom-left, as CameraController::takeClick reports). False when the ray
  // through the pixel misses the plane (parallel to it, or the plane is behind the camera).
  bool pick_ground(int x, int y, double ground_z, double out[3]) const;

  // Draw each peer's display name above its marker, into the current ImGui frame's background
  // draw list -- over the 3D view, under the UI's windows (call it from inside the frame, e.g.
  // ImGuiOverlay::setSceneDrawCallback, which runs even while the panels are hidden). Names come
  // from the room roster; a peer with no name shows its pid. Skips markers behind the camera. A
  // no-op without CVC_ENABLE_IMGUI.
  void draw_labels() const;

  // The room scene revision currently realized (0 = none yet).
  std::uint64_t realized_revision() const { return _realized_rev; }

  // Non-fatal realize warnings since the last call (a missing mesh, ...), then cleared.
  std::vector<std::string> take_warnings();

  // The pids that have an avatar marker right now, in pid order.
  std::vector<std::string> avatar_pids() const;

  // The SceneGraph node ids room_view uses, for hosts and tests.
  static std::string object_node_id(const std::string &id) { return "room_" + id; }
  static std::string avatar_node_id(const std::string &pid) { return "avatar_" + pid; }

  // Seconds over which a local reconcile correction is eased out (1/e).
  double correction_tau = 0.12;

private:
  void sync_scene();
  void clear_scene();
  void sync_avatars(cvc::ariadne::holon &h, double wall_dt);
  void sync_names();
  void select_lod();

  SceneRenderer &_view;
  cvc::app &_app;
  std::string _room, _self, _prefix;
  std::uint64_t _realized_rev = 0;
  std::unique_ptr<RealizedScene> _realized; // owns the scene's light rigs while shown
  std::vector<std::string> _scene_ids;      // the realized room nodes + lights, to remove
  // The mesh objects' pyramids, across re-realizes. It holds the nodes weakly and starts its build
  // thread only when the first mesh needs a build.
  std::unique_ptr<LodPyramidCache> _lod_cache;
  std::unique_ptr<cvc::gl::lod_stats> _lod_stats; // the last update's selection
  cvc::lod::quality_preset _lod_preset;
  std::map<std::string, bool> _avatars;             // pid -> marker created
  std::map<std::string, std::string> _names;        // pid -> display name (the roster), for labels
  std::map<std::string, cvc::physics::vec3> _heads; // pid -> where its label goes (world)
  int _names_age = 0;                               // frames until _names is re-read
  cvc::physics::vec3 _self_offset{0.0, 0.0, 0.0};   // easing offset on the local avatar
  bool _have_self = false;
  double _self_pos[3] = {0.0, 0.0, 0.0};
  std::vector<std::string> _warnings;
};

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_ROOM_VIEW_H
