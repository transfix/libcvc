#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <cvc/volren/camera.h>

namespace cvc {
namespace volren {

view_basis camera::basis() const {
  if (width <= 0 || height <= 0)
    throw volren_error("camera raster must be positive");
  if (width > limits::max_raster_dim || height > limits::max_raster_dim)
    throw volren_error("camera raster exceeds limits::max_raster_dim");
  for (int i = 0; i < 3; ++i)
    if (!std::isfinite(eye[i]) || !std::isfinite(focal[i]) || !std::isfinite(up[i]))
      throw volren_error("camera pose must be finite");
  if (projection == projection_type::perspective && !(vfov_degrees > 0.0 && vfov_degrees < 180.0))
    throw volren_error("vertical fov must be in (0, 180) degrees");
  if (projection == projection_type::orthographic &&
      !(parallel_scale > 0.0 && std::isfinite(parallel_scale)))
    throw volren_error("parallel_scale must be positive and finite");

  const vec3d back = normalized(vec3d(eye) - vec3d(focal));
  if (dot(back, back) == 0.0)
    throw volren_error("camera eye and focal point coincide (or are closer than epsilon)");
  const vec3d right = normalized(cross(vec3d(up), back));
  if (dot(right, right) == 0.0)
    throw volren_error("camera up vector is zero or parallel to the view direction");
  return {right, cross(back, right), back};
}

ray camera::generate_ray(int px, int py) const {
  const view_basis b = basis();
  const vec3d forward = -b.back;

  // NDC through the pixel center; v = +1 at the TOP row (cvc::image origin).
  const double u = (double(px) + 0.5) / double(width) * 2.0 - 1.0;
  const double v = 1.0 - (double(py) + 0.5) / double(height) * 2.0;

  if (projection == projection_type::perspective) {
    const double tan_half =
        std::tan(0.5 * vfov_degrees * boost::math::constants::pi<double>() / 180.0);
    const vec3d dir =
        normalized(forward + b.right * (u * tan_half * aspect()) + b.true_up * (v * tan_half));
    return {vec3d(eye), dir};
  }
  const vec3d origin =
      vec3d(eye) + b.right * (u * parallel_scale * aspect()) + b.true_up * (v * parallel_scale);
  return {origin, forward};
}

camera camera::from_pose(const double eye_[3], const double focal_[3], const double up_[3],
                         double vfov_degrees_, int width_, int height_) {
  camera c;
  c.eye = {eye_[0], eye_[1], eye_[2]};
  c.focal = {focal_[0], focal_[1], focal_[2]};
  c.up = {up_[0], up_[1], up_[2]};
  c.vfov_degrees = vfov_degrees_;
  c.width = width_;
  c.height = height_;
  return c;
}

double depth_to_window_z(double depth, double near_z, double far_z,
                         camera::projection_type projection) {
  if (!(far_z > near_z) || near_z <= 0.0)
    throw volren_error("depth_to_window_z needs 0 < near < far");
  const double d = std::min(std::max(depth, near_z), far_z);
  if (projection == camera::projection_type::orthographic)
    return (d - near_z) / (far_z - near_z);
  // Standard OpenGL perspective depth: z_ndc = (f+n)/(f-n) - 2fn/(d(f-n)),
  // window z = z_ndc/2 + 1/2, which simplifies to the form below.  The final
  // clamp keeps rounding from pushing the result epsilon past 1.0 at d==far.
  return std::min(1.0, std::max(0.0, far_z / (far_z - near_z) * (1.0 - near_z / d)));
}

} // namespace volren
} // namespace cvc
