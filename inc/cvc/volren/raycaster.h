// The cvc::volren renderer front-end.
//
// Replaces the legacy VolRenEnv/MultiVolRenEnv aggregate: volumes are
// cvc::volume shallow copies whose buffers are pinned (active_storage) for
// the duration of a render, settings are plain value types, and the render
// itself is const over all of them -- no per-thread env copies, no scratch
// smuggled through shared structs.
#ifndef CVC_VOLREN_RAYCASTER_H
#define CVC_VOLREN_RAYCASTER_H

#include <cstddef>
#include <cstdint>
#include <cvc/image/image.h>
#include <cvc/volren/camera.h>
#include <cvc/volren/settings.h>
#include <cvc/volren/types.h>
#include <cvc/volume/volume.h>
#include <map>
#include <memory>
#include <vector>

namespace cvc {

class app;
class thread_pool;

namespace volren {

// One rendered frame.
struct frame {
  // RGBA8, top-left origin (cvc::image convention); alpha is the ray's
  // accumulated opacity -- 0 where the ray missed every volume.
  cvc::image color;
  // GRAY f32: eye-space depth (distance along the view direction) of the
  // first isosurface hit or of the first sample that pushed accumulated
  // alpha past render_settings::depth_alpha_threshold; +inf where neither
  // happened.  Feed through depth_to_window_z() for a gl_FragDepth write.
  cvc::image depth;
};

// Which marcher render() runs.
//   cpu       -- the portable software march (the DEFAULT: every existing
//                behavior, test and byte-level guarantee is the CPU path's).
//   cuda      -- require the GPU path; render() throws cvc::volren_error when
//                there is no device or the scene is outside the v1 CUDA scope
//                (see raycaster_cuda.h).
//   automatic -- use the GPU when it is available AND the scene fits, else
//                fall back to the CPU silently.
// Either GPU mode falls back to the CPU (recording it in backend_used()) when
// the device path itself fails with cvc::cuda_error.
enum class backend { automatic, cpu, cuda };

class raycaster {
public:
  explicit raycaster(cvc::app &ctx);
  ~raycaster();

  // Register a volume (shallow copy -- cheap, shares the voxel buffer).
  // Throws cvc::volren_error for an empty or degenerate volume (any axis
  // with fewer than 2 voxels or a zero-extent bounding box).
  // Returns the volume's index for volume_config().
  // Note: CUDA-resident volumes should have min()/max() cached (or explicit
  // tf domains) -- the lazy min/max scan reads the host buffer.
  std::size_t add_volume(const cvc::volume &vol, volume_settings vs = volume_settings());
  void clear_volumes();
  std::size_t volume_count() const { return _volumes.size(); }
  // References are invalidated by add_volume()/clear_volumes().
  volume_settings &volume_config(std::size_t index);
  const volume_settings &volume_config(std::size_t index) const;

  // Announce that volume `index`'s voxels changed UNDER THE SAME BUFFER.
  //
  // Only the CUDA backend cares: it keeps voxel blocks resident on the device
  // between renders, keyed on the host pointer (raycaster_cuda.h spells out
  // the rule).  Because the cache co-owns the buffer, every write through the
  // supported cvc::volume API copy-on-writes to a NEW buffer and is picked up
  // automatically; the one case that needs announcing is an in-place write
  // through the unchecked legacy escape hatch voxels::data_ptr().  Calling it
  // when nothing changed is harmless -- it costs one re-upload.
  void invalidate_device_volume(std::size_t index);
  void invalidate_device_volumes();

  camera &view() { return _camera; }
  const camera &view() const { return _camera; }
  render_settings &settings() { return _settings; }
  const render_settings &settings() const { return _settings; }

  // Borrow a pool for tile parallelism.  By default (nullptr) the raycaster
  // lazily creates its OWN pool rather than borrowing ctx.computePool():
  // cvc::thread_pool supports only one in-flight parallel_for per pool, so
  // sharing the app-wide pool with another fan-out (e.g. cvc::nav) can
  // deadlock.  Inject a pool only when nothing else runs parallel_for on it
  // concurrently.  render_settings::threads == 1 forces serial.
  void set_thread_pool(cvc::thread_pool *pool) { _pool = pool; }

  // Backend selection.  Defaults to backend::cpu, so the GPU path is strictly
  // opt-in and no existing caller changes behavior.
  void set_backend(backend b) { _backend = b; }
  backend backend_selected() const { return _backend; }
  // What the LAST render() actually ran on (backend::cpu before any render,
  // and after a cvc::cuda_error fallback).
  backend backend_used() const { return _backend_used; }

  // Union of the registered volumes' bounding boxes (the legacy metavolume,
  // without its zero-initialized-union bug).  Throws cvc::volren_error when
  // no volumes are registered.
  cvc::bounding_box scene_bounds() const;

  // ---- volumetric shadows (shadow.h) --------------------------------------
  // The light-view depth maps built by the LAST render(); empty when shadows
  // are off, and empty for a light whose direction is degenerate.  Same
  // contract as frame::depth: GRAY f32, eye-space depth measured from the
  // light's orthographic eye plane, +inf where the light ray hit nothing.
  std::size_t shadow_map_count() const { return _shadow_views.size(); }
  // All three throw cvc::volren_error for an out-of-range index.
  cvc::image shadow_map_depth(std::size_t i) const;
  const shadow_view &shadow_map_view(std::size_t i) const;
  // The DEEP transmittance profile of map `i` (shadow_mode::deep), or an empty
  // image for a hard map.  GRAY f32, PLANE-major: `shadow_view::slices + 1`
  // planes of the light-view raster stacked into one image, so
  //   width  = shadow_view::width
  //   height = shadow_view::height * (slices + 1)
  //   plane j, texel (x, y) = index j * width * height + y * width + x
  // detail::shadow_visibility_deep documents each plane's contents and why the
  // layout is plane-major.  Public for the same reason shadow_view is: a
  // consumer can evaluate the map at its OWN points (a ground decal, a debug
  // overlay) without reaching into detail/.
  cvc::image shadow_map_profile(std::size_t i) const;

  // The maps are cached and rebuilt only when the light pass's own inputs
  // change (lights, volumes, model transforms, per-volume settings, cut
  // planes, steps, scene bounds, shadow settings) -- NOT when the camera
  // moves, which is what makes shadows free while the user orbits.  Mutating a
  // registered volume's voxels IN PLACE is invisible to that fingerprint (the
  // buffer pointer and length are what identify it); call this after doing so.
  void invalidate_shadow_maps();

  // Render one frame.  Reports progress through the app thread map
  // (cvc::app::threadProgress) and honors boost thread interruption between
  // tiles.  Deterministic: output is byte-identical for any thread count.
  frame render();

private:
  cvc::app &_ctx;
  camera _camera;
  render_settings _settings;
  std::vector<cvc::volume> _volumes;
  std::vector<volume_settings> _volume_settings;
  // Content generation per volume, handed to the CUDA backend's resident
  // cache.  Drawn from a process-wide counter so a bump is unambiguous even
  // when two raycasters share a voxel buffer.
  std::vector<std::uint64_t> _volume_generation;
  // Invalidations announced for a host buffer, kept across clear_volumes() so
  // an announce-then-re-register cycle does not silently rewind the stamp.
  // Keyed on the voxel buffer, which is what the device cache is keyed on.
  std::map<const void *, std::uint64_t> _announced_generation;
  cvc::thread_pool *_pool = nullptr;           // borrowed, may be null
  std::unique_ptr<cvc::thread_pool> _own_pool; // lazy default
  backend _backend = backend::cpu;
  backend _backend_used = backend::cpu;

  // Cached light-view depth maps, one per shadow-casting light, plus the
  // fingerprint of the inputs they were built from (0 == nothing built).
  // _shadow_profile is parallel to them and empty in shadow_mode::hard.
  std::vector<shadow_view> _shadow_views;
  std::vector<cvc::image> _shadow_depth;
  std::vector<cvc::image> _shadow_profile;
  std::uint64_t _shadow_key = 0;

  // ---- deep-shadow profile capture (INTERNAL) -----------------------------
  // The transmittance profile is a new OUTPUT of the marcher, but it is not a
  // user-facing render setting: it is meaningful only for the orthographic
  // light-view pass, whose ray parameter t IS the light-space depth.  So it is
  // requested by ensure_shadow_maps() writing this on the temporary light-pass
  // raycaster (same class, so private access is legitimate) and read back out
  // of _profile_out, instead of widening render_settings and frame for a mode
  // no caller can use correctly on its own camera.
  //
  // slices == 0 (the default, and every user-driven render) makes render() take
  // exactly the code path it took before deep shadows existed.
  struct profile_request {
    int slices = 0;
    double z_near = 0.0; // light-space depth of knot 0
    double dz = 0.0;     // knot spacing
  };
  profile_request _profile_req;
  cvc::image _profile_out; // filled by render() iff _profile_req.slices > 0

  // Rebuild _shadow_views/_shadow_depth if `scene` or any light-pass input
  // moved since they were built.  Called by render() before it marches.
  void ensure_shadow_maps(const cvc::bounding_box &scene);
};

} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_RAYCASTER_H
