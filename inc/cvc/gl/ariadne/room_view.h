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
//     as a glide rather than a jump.
//   * Each marker carries a dark visor on its front face, so its facing (the pose rotation, from
//     the participant's intent.yaw) reads at a glance; each peer's display name floats above it
//     (draw_labels, from the host's ImGuiOverlay scene-draw callback).
//   * Third-person controls: intent_from_keys turns held WASD / arrow keys into forward / back /
//     strafe relative to the avatar's FACING; the host turns the avatar by a click on the scene
//     (pick_ground -> face that point). self_position is where the host keeps the orbit centre.
//
// Owner/render thread, like the SceneGraph it draws into.

#include <cstdint>
#include <cvc/ariadne/avatar_sim.h> // avatar_intent
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
  ~room_view(); // removes every node it created
  room_view(const room_view &) = delete;
  room_view &operator=(const room_view &) = delete;

  // Once per frame, after client_pump advanced the holon: re-realize the room scene if its
  // revision changed, add/remove avatar markers to match the holon's participants, and pose them.
  // wall_dt (seconds) paces the correction easing.
  void update(cvc::ariadne::holon &h, double wall_dt);

  // Where the local avatar is shown (the eased display position, at the marker's feet). False
  // until the holon has the local avatar.
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

  SceneRenderer &_view;
  cvc::app &_app;
  std::string _room, _self, _prefix;
  std::uint64_t _realized_rev = 0;
  std::unique_ptr<RealizedScene> _realized;       // owns the scene's light rigs while shown
  std::vector<std::string> _scene_ids;            // the realized room nodes + lights, to remove
  std::map<std::string, bool> _avatars;           // pid -> marker created
  std::map<std::string, std::string> _names;      // pid -> display name (the roster), for labels
  std::map<std::string, cvc::physics::vec3> _heads; // pid -> where its label goes (world)
  int _names_age = 0;                               // frames until _names is re-read
  cvc::physics::vec3 _self_offset{0.0, 0.0, 0.0}; // easing offset on the local avatar
  bool _have_self = false;
  double _self_pos[3] = {0.0, 0.0, 0.0};
  std::vector<std::string> _warnings;
};

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_ROOM_VIEW_H
