// Camera model for cvc::volren.
//
// Deliberately parameterized the way cvcGL/VTK parameterize a camera --
// eye / focal point / view-up, VERTICAL field of view in degrees, and
// vtkCamera::SetParallelScale semantics for orthographic projection -- so
// `CameraController::getPose()` + `vtkCamera::GetViewAngle()` feed straight
// into a raycast pass and the resulting frame can be composited into a cvcGL
// scene with a depth-aware shader.
#ifndef CVC_VOLREN_CAMERA_H
#define CVC_VOLREN_CAMERA_H

#include <cvc/volren/types.h>

namespace cvc {
namespace volren {

// A world-space ray through a pixel center.  `direction` is unit length and
// points INTO the scene (unlike the legacy Viewing::raydir, which pointed at
// the eye and was mutated per ray through the shared env).
struct ray {
  vec3d origin;
  vec3d direction;
};

// Right-handed orthonormal camera frame.  `back` points from the focal point
// toward the viewer (the legacy vpn); the view direction is -back.
struct view_basis {
  vec3d right;
  vec3d true_up;
  vec3d back;
};

struct camera {
  enum class projection_type { perspective, orthographic };

  // Default pose: south of the focal point looking north, Z-up (an eye on the
  // +Z axis would make `up` parallel to the view direction and basis() throw).
  std::array<double, 3> eye{0.0, -4.0, 0.0};
  std::array<double, 3> focal{0.0, 0.0, 0.0};
  std::array<double, 3> up{0.0, 0.0, 1.0}; // cvc scenes are Z-up
  projection_type projection = projection_type::perspective;
  double vfov_degrees = defaults::vfov_degrees; // vertical, like vtkCamera::ViewAngle
  double parallel_scale = 1.0; // ortho half-height in world units (vtkCamera::SetParallelScale)
  int width = 512;
  int height = 512;

  double aspect() const { return height > 0 ? double(width) / double(height) : 1.0; }

  // Throws cvc::volren_error on a degenerate pose (eye == focal, zero up,
  // up parallel to the view direction, or a non-positive raster/fov).
  view_basis basis() const;

  // The ray through the CENTER of pixel (px, py).  Pixel (0,0) is the
  // TOP-LEFT of the image, matching cvc::image's origin (use
  // image::flipped_vertical() when handing frames to GL/VTK).
  ray generate_ray(int px, int py) const;

  static camera from_pose(const double eye[3], const double focal[3], const double up[3],
                          double vfov_degrees, int width, int height);
};

// Convert an eye-space depth (distance along the view direction, what
// frame::depth stores) to an OpenGL window-space z in [0,1] for the given
// clip range -- the value a `gl_FragDepth` write needs when compositing a
// raycast frame into a GL scene.  Depths outside [near_z, far_z] clamp.
double depth_to_window_z(double depth, double near_z, double far_z,
                         camera::projection_type projection);

} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_CAMERA_H
