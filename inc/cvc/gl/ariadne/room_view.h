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
//   * Third-person controls: intent_from_keys turns held WASD / arrow keys into an intent relative
//     to the camera's heading, and self_position is where the host keeps the camera's orbit centre.
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

// The local avatar's intent from the held movement keys, relative to the camera's heading on a
// Z-up ground: forward is the view direction (eye -> focal) flattened onto the ground, right is
// forward x up. Diagonals are normalized (never faster than `speed`); opposing keys cancel. A
// camera looking straight down has no heading, so forward is then +Y.
cvc::ariadne::avatar_intent intent_from_keys(bool forward, bool back, bool left, bool right,
                                             const double eye[3], const double focal[3],
                                             double speed);

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

  // The room scene revision currently realized (0 = none yet).
  std::uint64_t realized_revision() const { return _realized_rev; }

  // Non-fatal realize warnings since the last call (a missing mesh, ...), then cleared.
  std::vector<std::string> take_warnings();

  // The SceneGraph node ids room_view uses, for hosts and tests.
  static std::string object_node_id(const std::string &id) { return "room_" + id; }
  static std::string avatar_node_id(const std::string &pid) { return "avatar_" + pid; }

  // Seconds over which a local reconcile correction is eased out (1/e).
  double correction_tau = 0.12;

private:
  void sync_scene();
  void clear_scene();
  void sync_avatars(cvc::ariadne::holon &h, double wall_dt);

  SceneRenderer &_view;
  cvc::app &_app;
  std::string _room, _self, _prefix;
  std::uint64_t _realized_rev = 0;
  std::unique_ptr<RealizedScene> _realized;       // owns the scene's light rigs while shown
  std::vector<std::string> _scene_ids;            // the realized room nodes + lights, to remove
  std::map<std::string, bool> _avatars;           // pid -> marker created
  cvc::physics::vec3 _self_offset{0.0, 0.0, 0.0}; // easing offset on the local avatar
  bool _have_self = false;
  double _self_pos[3] = {0.0, 0.0, 0.0};
  std::vector<std::string> _warnings;
};

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_ROOM_VIEW_H
