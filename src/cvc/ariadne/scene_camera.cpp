// A camera's authored viewpoint (§5.18, Cameras): the keys a `scene: cameras:` entry and a
// viewport's own camera share — see scene.h.
#include <cmath>
#include <cvc/ariadne/scene.h>
#include <limits>

namespace cvc {
namespace ariadne {

namespace {

// [x, y, z] of numbers under `key`: true and `out` set when all three are numbers.
bool vec3_of(const Value &v, const char *key, double out[3]) {
  const Value *x = v.find(key);
  if (!x || !x->is_seq() || x->items.size() != 3)
    return false;
  double r[3];
  for (int i = 0; i < 3; ++i) {
    r[i] = x->items[i].as_double(std::numeric_limits<double>::quiet_NaN()); // NaN: no number
    if (!std::isfinite(r[i]))
      return false;
  }
  for (int i = 0; i < 3; ++i)
    out[i] = r[i];
  return true;
}

} // namespace

void read_scene_camera_viewpoint(const Value &v, SceneCamera &cam) {
  if (!v.is_map())
    return;
  if (vec3_of(v, "position", cam.position))
    cam.has_position = true;
  if (vec3_of(v, "target", cam.target))
    cam.has_target = true;
  double up[3];
  if (vec3_of(v, "up", up) && (up[0] != 0.0 || up[1] != 0.0 || up[2] != 0.0)) {
    for (int i = 0; i < 3; ++i)
      cam.up[i] = up[i];
    cam.has_up = true;
  }
  const double fov = v.num("fov", cam.fov);
  cam.fov = fov > 0.0 && fov < 180.0 ? fov : 0.0;
  cam.near_clip = v.num("near", cam.near_clip);
  cam.far_clip = v.num("far", cam.far_clip);
  cam.orthographic = v.flag("orthographic", cam.orthographic);
  cam.height = v.num("height", cam.height);
}

} // namespace ariadne
} // namespace cvc
