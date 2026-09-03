// CUDA backend for the cvc::volren raycaster.
//
// This is the device twin of raycaster.cpp's render_ray: one CUDA thread per
// pixel, the same ray generation / slab intersect / Amanatides-Woo isosurface
// DDA / per-cell transfer-function march / front-to-back compositing.  It is a
// SEMANTIC mirror, not a bit-exact one -- raycast.cu is compiled with
// --use_fast_math in Release (src/cvc/CMakeLists.txt puts volume rendering in
// the "no bit/float-equivalence contract" class), so parity is an image-level
// property, not a float-equality one.
//
// Declarations are UNCONDITIONAL: a build without CUDA still links, because
// raycaster_cuda_stub.cpp supplies `available() == false` + a throwing
// raycast_cuda().  Guard every call site with raycast_cuda_available().
//
// v1 scope limits (the raycaster falls back to the CPU path, or throws for an
// explicit backend::cuda, when a scene exceeds them):
//   - exactly ONE volume (the CPU path composites an arbitrary number);
//   - at most cuda_limits::max_isosurfaces isosurfaces on that volume;
//   - at most cuda_limits::max_lights lights and cuda_limits::max_cut_planes
//     cut planes in the scene;
//   - at most cuda_limits::max_iso_hits_per_ray isosurface hits kept per ray
//     (the nearest ones win -- see raycast.cu).
// The volume buffer and the baked LUT are uploaded per render(); a resident
// device-side volume cache is future work.
#ifndef CVC_VOLREN_RAYCASTER_CUDA_H
#define CVC_VOLREN_RAYCASTER_CUDA_H

#include <cstdint>
#include <cvc/core/types.h>
#include <cvc/volren/camera.h>
#include <cvc/volren/raycaster.h>
#include <cvc/volren/types.h>

namespace cvc {
namespace volren {

// Fixed caps of the v1 device path.  They are compile-time because the kernel
// carries the corresponding arrays by value in its parameter block and in
// per-thread registers/local memory -- no device-side allocation, no
// indirection, and a bounded local footprint per ray.
namespace cuda_limits {
inline constexpr int max_isosurfaces = 8;
inline constexpr int max_lights = 8;
inline constexpr int max_cut_planes = 8;
// Per-ray isosurface hit buffer.  Rays crossing more surfaces than this keep
// the NEAREST hits and silently drop the rest -- unlike the other limits here
// this one cannot be range-checked up front (the crossing count is not known
// until the ray is traced), so a scene of many low-opacity isosurfaces can
// diverge slightly from the CPU image.  See push_hit() in raycast.cu.
inline constexpr int max_iso_hits_per_ray = 32;
} // namespace cuda_limits

// Flattened mirrors of the settings.h value types: fixed-size C arrays only,
// so the whole request is trivially copyable into the kernel parameter block.
struct cuda_isosurface {
  double value = 0.0;
  float opacity = 1.0f;
  float color[3] = {1.f, 1.f, 1.f};
  float shininess = defaults::shininess;
};

struct cuda_light {
  float color[3] = {1.f, 1.f, 1.f};
  double direction[3] = {0.0, 0.0, 1.0}; // toward the light; normalized on use
};

struct cuda_cut_plane {
  double point[3] = {0.0, 0.0, 0.0};
  double normal[3] = {0.0, 0.0, 1.0}; // points at the KEPT half-space
};

// Everything one device render needs, resolved on the host exactly the way
// raycaster::render() resolves it for the CPU march.
struct raycast_cuda_request {
  // ---- view ---------------------------------------------------------------
  camera cam; // reused verbatim; the kernel expands it like ray_generator

  // ---- the single volume --------------------------------------------------
  // HOST pointer to the raw voxel buffer (the caller must keep it pinned for
  // the duration of the call -- voxels::active_storage()).  Uploaded here.
  const void *data = nullptr;
  cvc::data_type type = cvc::UChar;
  std::int64_t dimx = 0, dimy = 0, dimz = 0;
  double minb[3] = {0.0, 0.0, 0.0}; // world position of voxel (0,0,0)
  double span[3] = {1.0, 1.0, 1.0}; // voxel spacing per axis (all > 0)

  // World -> volume-local affine inverse of volume_settings::model_transform,
  // row-major (types.h mat4 storage).  Ignored unless `transformed`.
  bool transformed = false;
  double world_to_local[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

  // ---- baked transfer function -------------------------------------------
  // `lut` is a HOST array of lut_size RGBA quads (4 floats each) uniform over
  // [tf_lo, tf_hi]; nearest-entry lookup, NaN routed to entry 0 -- the
  // baked_transfer_function::sample() contract.  Inactive when the volume is
  // neither shaded nor unshaded, or when the bake was degenerate (empty LUT).
  bool tf_active = false;
  const float *lut = nullptr;
  int lut_size = 0;
  double tf_lo = 0.0, tf_hi = 1.0;

  // ---- gradient-magnitude opacity ramp (transfer_function.h) --------------
  bool gradient_ramp_enabled = false;
  double ramp0 = 0.0, ramp1 = 0.0, ramp2 = 0.0;
  double gradient_plateau = defaults::gradient_plateau;

  // ---- scene contents (fixed caps) ---------------------------------------
  int isosurface_count = 0;
  cuda_isosurface isosurfaces[cuda_limits::max_isosurfaces];
  int light_count = 0;
  cuda_light lights[cuda_limits::max_lights];
  int cut_plane_count = 0;
  cuda_cut_plane cut_planes[cuda_limits::max_cut_planes];

  // ---- march / compositing ------------------------------------------------
  double scene_min[3] = {0.0, 0.0, 0.0}; // scene_bounds(), the marched AABB
  double scene_max[3] = {0.0, 0.0, 0.0};
  int steps = defaults::steps;
  double unit_step = 0.0; // diagonal(scene_bounds) / steps, computed by the caller
  float opacity_cutoff = defaults::opacity_cutoff;
  float depth_alpha_threshold = defaults::depth_alpha_threshold;
  float ambient = 0.0f;
  bool two_sided = false;
  float background[3] = {0.f, 0.f, 0.f};
  // Shininess used for shaded TF samples (isosurfaces carry their own).
  float tf_shininess = defaults::shininess;

  // ---- per-volume mode + density window ----------------------------------
  bool shaded = true;
  bool unshaded = false;
  bool window_enabled = false;
  double window_min = 0.0;
  double window_max = 0.0;
};

// False when libcvc was built without CUDA, or when no CUDA device is usable.
// Never throws.
bool raycast_cuda_available();

// Render one frame on the GPU.  Throws cvc::volren_error for an invalid
// request (bad raster, degenerate volume, unsupported data type) and
// cvc::cuda_error for any device-side failure -- callers that want a CPU
// fallback catch the latter.
frame raycast_cuda(const raycast_cuda_request &req);

} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_RAYCASTER_CUDA_H
