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
#include <cvc/image/image.h>
#include <cvc/volren/camera.h>
#include <cvc/volren/settings.h>
#include <cvc/volren/types.h>
#include <cvc/volume/volume.h>
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

  // Union of the registered volumes' bounding boxes (the legacy metavolume,
  // without its zero-initialized-union bug).  Throws cvc::volren_error when
  // no volumes are registered.
  cvc::bounding_box scene_bounds() const;

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
  cvc::thread_pool *_pool = nullptr;           // borrowed, may be null
  std::unique_ptr<cvc::thread_pool> _own_pool; // lazy default
};

} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_RAYCASTER_H
