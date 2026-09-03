// CUDA backend for the cvc::volren raycaster.
//
// This is the device twin of raycaster.cpp's render_ray: one CUDA thread per
// pixel, the same ray generation / slab intersect / per-ray volume cull /
// Amanatides-Woo isosurface DDA / per-cell transfer-function march /
// front-to-back compositing.  It is a SEMANTIC mirror, not a bit-exact one --
// raycast.cu is compiled with --use_fast_math in Release (src/cvc/CMakeLists.txt
// puts volume rendering in the "no bit/float-equivalence contract" class), so
// parity is an image-level property, not a float-equality one.
//
// Declarations are UNCONDITIONAL: a build without CUDA still links, because
// raycaster_cuda_stub.cpp supplies `available() == false` + a throwing
// raycast_cuda(). Guard every call site with raycast_cuda_available().
//
// Scope limits (the raycaster falls back to the CPU path, or throws for an
// explicit backend::cuda, when a scene exceeds them):
//   - at most cuda_limits::max_volumes volumes, each with at most
//     cuda_limits::max_isosurfaces isosurfaces;
//   - at most cuda_limits::max_lights lights and cuda_limits::max_cut_planes
//     cut planes in the scene;
//   - at most cuda_limits::max_iso_hits_per_ray isosurface hits kept per ray
//     ACROSS all volumes (the nearest ones win -- see raycast.cu);
//   - at most cuda_limits::max_shadow_maps light-view shadow maps, which
//     raycaster::render() already enforces for both backends.
//
// The light-view passes that BUILD the shadow maps are ordinary render()
// calls, so they pick their own backend; this entry point only consumes the
// finished depth rasters.
//
// Volume voxels are uploaded once and kept RESIDENT (see "Device volume
// cache" below), so a camera-only re-render launches with no H2D traffic; the
// baked LUTs are small and are re-staged per render.
#ifndef CVC_VOLREN_RAYCASTER_CUDA_H
#define CVC_VOLREN_RAYCASTER_CUDA_H

#include <cstddef>
#include <cstdint>
#include <cvc/core/types.h>
#include <cvc/volren/camera.h>
#include <cvc/volren/raycaster.h>
#include <cvc/volren/types.h>
#include <memory>

namespace cvc {
namespace volren {

// Fixed caps of the device path.  They are compile-time because the kernel
// carries the corresponding arrays either by value in its parameter block or
// in per-thread registers/local memory -- no device-side allocation and a
// bounded local footprint per ray.
namespace cuda_limits {
inline constexpr int max_isosurfaces = 8;
inline constexpr int max_lights = 8;
inline constexpr int max_cut_planes = 8;
// Per-ray isosurface hit buffer, shared by every volume the ray crosses (the
// CPU path likewise merges all volumes' hits into one t-ordered stream).  Rays
// crossing more surfaces than this keep the NEAREST hits and silently drop the
// rest -- unlike the other limits here this one cannot be range-checked up
// front (the crossing count is not known until the ray is traced), so a scene
// of many low-opacity isosurfaces can diverge slightly from the CPU image.
// See push_hit() in raycast.cu.
inline constexpr int max_iso_hits_per_ray = 32;
// Volumes carried by one device render.
//
// Why 16 and not more: the per-volume block (its transform, LUT descriptor and
// isosurface table) is ~600 bytes, so 16 of them no longer fit in the 4 KB
// kernel parameter limit -- they live in device memory and the kernel takes a
// pointer (see dev_volume in raycast.cu).  What 16 really buys is bounded
// PER-THREAD state: the march needs a last-cell tracker and a cull window per
// volume, i.e. 16 * (3 * 8 + 2 * 8) = 640 bytes of local memory per ray on top
// of the spline cache and the hit buffer, for ~2.2 KB per thread.  Doubling
// the cap doubles that tail and costs occupancy on every scene, including
// single-volume ones; 16 covers the scenes this renderer is actually driven
// with (VolRenNode's 3x3 instancing grid tops out at 9) with headroom.
inline constexpr int max_volumes = 16;
// Default byte budget of the resident device volume cache.  One 512^3 UShort
// volume is 256 MB, so this holds a couple of large volumes or a whole grid of
// instanced small ones; raycast_cuda_set_cache_budget() overrides it.
inline constexpr std::size_t default_cache_bytes = std::size_t(1) << 29; // 512 MB
// Light-view shadow maps carried by one device render.  The kernel holds their
// light-view FRAMES by value in its parameter block (~128 bytes each, well
// inside the 4 KB budget alongside the existing ~1.5 KB request); only the
// depth rasters live in device memory.
inline constexpr int max_shadow_maps = 4;
// The renderer-level cap and the device cap are ONE contract, enforced by
// raycaster::render() before either backend runs -- so a scene with more
// casting lights than this is refused on the CPU path too, and the device path
// never sees one.  Raising limits::max_shadow_maps without raising this would
// make the CPU accept a scene the kernel cannot represent.
static_assert(max_shadow_maps == limits::max_shadow_maps,
              "cuda_limits::max_shadow_maps must match limits::max_shadow_maps");
} // namespace cuda_limits

// Flattened mirrors of the settings.h value types: fixed-size C arrays only.
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

// One built light-view shadow map: the flattened shadow_view plus a HOST
// pointer to its raw f32 depth raster.  The host stages every map's raster
// into one device allocation, so `depth` need only stay valid for the call.
struct cuda_shadow_map {
  double eye[3] = {0.0, 0.0, 0.0};   // on the light's eye PLANE
  double right[3] = {1.0, 0.0, 0.0}; // orthonormal light-view basis
  double up[3] = {0.0, 1.0, 0.0};
  double forward[3] = {0.0, 0.0, 1.0}; // depth grows along it
  double parallel_scale = 1.0;         // half-height AND half-width (aspect is 1)
  double texel_world = 0.0;            // 2 * parallel_scale / height
  int width = 0, height = 0;
  int light_index = -1;         // index into lights[]
  const float *depth = nullptr; // width*height floats, row-major, +inf on a miss
};

// One volume of the scene, resolved on the host exactly the way
// raycaster::render() resolves a prepared_volume for the CPU march.
struct cuda_volume {
  // ---- voxels -------------------------------------------------------------
  // HOST pointer to the raw voxel buffer.  `pin` must be the owning
  // voxels::active_storage() handle: it keeps the buffer alive for the call
  // AND, while the device cache holds the block, makes `data` a unique
  // identity for cache lookups (see the cache notes below).
  const void *data = nullptr;
  std::shared_ptr<void> pin;
  cvc::data_type type = cvc::UChar;
  std::int64_t dimx = 0, dimy = 0, dimz = 0;
  double minb[3] = {0.0, 0.0, 0.0}; // world position of voxel (0,0,0)
  double span[3] = {1.0, 1.0, 1.0}; // voxel spacing per axis (all > 0)
  // Bumped by the caller whenever `data`'s CONTENTS changed under a pointer
  // that stayed the same; a mismatch against the cached copy forces a
  // re-upload into the resident allocation.
  std::uint64_t generation = 0;

  // World -> volume-local affine inverse of volume_settings::model_transform,
  // row-major (types.h mat4 storage).  Ignored unless `transformed`.
  bool transformed = false;
  double world_to_local[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

  // ---- baked transfer function -------------------------------------------
  // `lut` is a HOST array of lut_size RGBA quads (4 floats each) uniform over
  // [tf_lo, tf_hi]; nearest-entry lookup, NaN routed to entry 0 -- the
  // baked_transfer_function::sample() contract.  Inactive when the volume is
  // neither shaded nor unshaded, or when the bake was degenerate.
  bool tf_active = false;
  const float *lut = nullptr;
  int lut_size = 0;
  double tf_lo = 0.0, tf_hi = 1.0;

  // ---- gradient-magnitude opacity ramp (transfer_function.h) --------------
  bool gradient_ramp_enabled = false;
  double ramp0 = 0.0, ramp1 = 0.0, ramp2 = 0.0;
  double gradient_plateau = defaults::gradient_plateau;

  // ---- per-volume mode + density window ----------------------------------
  int isosurface_count = 0;
  cuda_isosurface isosurfaces[cuda_limits::max_isosurfaces];
  bool shaded = true;
  bool unshaded = false;
  bool window_enabled = false;
  double window_min = 0.0;
  double window_max = 0.0;
};

// Everything one device render needs.
struct raycast_cuda_request {
  // ---- view ---------------------------------------------------------------
  camera cam; // reused verbatim; the kernel expands it like ray_generator

  // ---- volumes ------------------------------------------------------------
  int volume_count = 0;
  cuda_volume volumes[cuda_limits::max_volumes];

  // ---- scene contents (fixed caps) ---------------------------------------
  int light_count = 0;
  cuda_light lights[cuda_limits::max_lights];
  int cut_plane_count = 0;
  cuda_cut_plane cut_planes[cuda_limits::max_cut_planes];

  // ---- volumetric shadows (shadow.h) --------------------------------------
  // Zero maps means shadows are off, which is the pre-shadow kernel path.
  int shadow_map_count = 0;
  cuda_shadow_map shadow_maps[cuda_limits::max_shadow_maps];
  float shadow_strength = 1.0f;
  // bias_scale * latch_quantum, folded on the host because it is scene-scaled and
  // constant over the frame; the kernel adds only the per-sample slope term
  // shadow_slope_scale * texel_world * tan(theta).
  double shadow_bias_constant = 0.0;
  double shadow_slope_scale = 1.0;

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
  // Sub-samples per pixel EDGE (render_settings::supersample), in
  // [1, limits::max_supersample].  One thread still owns one PIXEL and simply
  // marches supersample^2 rays through it, so nothing about the device path's
  // per-thread footprint or its scope limits depends on this value.
  int supersample = defaults::supersample;
};

// False when libcvc was built without CUDA, or when no CUDA device is usable.
// Never throws.
bool raycast_cuda_available();

// Render one frame on the GPU.  Throws cvc::volren_error for an invalid
// request (bad raster, degenerate volume, unsupported data type) and
// cvc::cuda_error for any device-side failure -- callers that want a CPU
// fallback catch the latter.
frame raycast_cuda(const raycast_cuda_request &req);

// ---------------------------------------------------------------------------
// Device volume cache
// ---------------------------------------------------------------------------
// Voxel blocks stay resident on the device across renders and across raycaster
// instances, keyed on (device, host pointer, byte length) and validated
// against cuda_volume::generation.  The entry co-owns the host block through
// cuda_volume::pin, which is what makes the key sound: the block cannot be
// freed and its address cannot be recycled
// by a different volume while it is cached, and every write through the
// supported cvc::volume API copy-on-writes to a DIFFERENT block precisely
// because the cache is a co-owner -- so a mutated volume is a cache miss by
// construction.  The one uncovered path is an in-place write through the
// unchecked legacy escape hatch (voxels::data_ptr()), which the owner must
// announce with raycaster::invalidate_device_volume().
//
// Entries are evicted least-recently-used once the resident total would exceed
// the budget; an entry in use by an in-flight render is never evicted.
void raycast_cuda_set_cache_budget(std::size_t bytes);
std::size_t raycast_cuda_cache_budget();
// Bytes currently resident across every device (0 without CUDA).
std::size_t raycast_cuda_cache_bytes();
// Voxel bytes actually pushed host-to-device since process start.  A render
// that only moved the camera adds nothing to it -- which is the whole claim
// the cache makes, so it is worth being able to assert.
std::uint64_t raycast_cuda_cache_upload_bytes();
// Free every cached block not in use by an in-flight render.  Safe to call
// without a device; never throws.
void raycast_cuda_clear_cache();

} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_RAYCASTER_CUDA_H
