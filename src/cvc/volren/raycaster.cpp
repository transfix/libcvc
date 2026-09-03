#include <algorithm>
#include <atomic>
#include <boost/current_function.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/thread/thread.hpp>
#include <cmath>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/thread_pool.h>
#include <cvc/utility/cuda_utils.h>
#include <cvc/volren/detail/cell_intersect.h>
#include <cvc/volren/detail/sampler.h>
#include <cvc/volren/detail/shading.h>
#include <cvc/volren/detail/shadow_map.h>
#include <cvc/volren/detail/spline_gradient.h>
#include <cvc/volren/raycaster.h>
#include <cvc/volren/raycaster_cuda.h>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

namespace cvc {
namespace volren {

namespace {

// Everything the march needs per volume, resolved once per render() so the
// per-ray loop touches only raw buffers and baked tables.  The `pin` keeps
// the voxel storage alive (and the data pointer valid) even if the caller
// mutates or drops its volume mid-render.
struct prepared_volume {
  detail::grid_sampler grid;
  std::shared_ptr<void> pin;
  const volume_settings *vs = nullptr;
  baked_transfer_function tf;
  bool any_iso = false;
  // Scene-graph placement: world -> volume-local inverse of the model matrix.
  bool transformed = false;
  mat4 world_to_local;
  // Local-space CULL box for the per-ray active-volume test.  grid_sampler::
  // cell_index() can only succeed for local x in (minb - span, minb + span *
  // (dim-1)) -- the truncation-toward-zero entry slack on the low side and the
  // idx <= dim-2 clamp on the high side -- so a box CONTAINING that region can
  // reject a volume for a whole ray without changing a single sample.  One
  // extra voxel of margin per face keeps the rejection immune to the rounding
  // difference between "solve for t on the face" and "divide the marched point
  // by span", which is what makes the culling byte-identical rather than
  // merely close.
  vec3d cull_lo, cull_hi;

  vec3d to_local_point(const vec3d &p) const {
    return transformed ? world_to_local.transform_point(p) : p;
  }
  vec3d to_local_vector(const vec3d &v) const {
    return transformed ? world_to_local.transform_vector(v) : v;
  }
  // Local gradient/normal -> world: transpose(inverse(A)) * n, using the
  // already-inverted linear part (transpose of world_to_local's).
  vec3d normal_to_world(const vec3d &n) const {
    if (!transformed)
      return n;
    const std::array<double, 16> &i = world_to_local.m;
    return {i[0] * n.x + i[4] * n.y + i[8] * n.z, i[1] * n.x + i[5] * n.y + i[9] * n.z,
            i[2] * n.x + i[6] * n.y + i[10] * n.z};
  }
};

// Camera expanded for the inner loop: basis and fov factors computed once,
// not per ray (camera::generate_ray revalidates per call; same math).
struct ray_generator {
  vec3d eye, right, true_up, forward;
  bool perspective = true;
  double tan_half = 1.0, parallel_scale = 1.0, aspect = 1.0;
  int width = 0, height = 0;

  explicit ray_generator(const camera &cam) {
    const view_basis b = cam.basis();
    eye = vec3d(cam.eye);
    right = b.right;
    true_up = b.true_up;
    forward = -b.back;
    perspective = cam.projection == camera::projection_type::perspective;
    tan_half = std::tan(0.5 * cam.vfov_degrees * boost::math::constants::pi<double>() / 180.0);
    parallel_scale = cam.parallel_scale;
    aspect = cam.aspect();
    width = cam.width;
    height = cam.height;
  }

  // The ray through a point INSIDE pixel (px, py): `ox`/`oy` are sub-pixel
  // offsets in [0, 1) from the pixel's top-left corner, so (0.5, 0.5) is the
  // pixel center.  Supersampling passes ((i+0.5)/n, (j+0.5)/n); with n == 1
  // that offset IS 0.5 exactly (0.5 / 1.0 is exact in IEEE), so the single-
  // sample path evaluates the same expression it always did, bit for bit.
  ray at(int px, int py, double ox, double oy) const {
    const double u = (double(px) + ox) / double(width) * 2.0 - 1.0;
    const double v = 1.0 - (double(py) + oy) / double(height) * 2.0;
    if (perspective) {
      return {eye,
              normalized(forward + right * (u * tan_half * aspect) + true_up * (v * tan_half))};
    }
    return {eye + right * (u * parallel_scale * aspect) + true_up * (v * parallel_scale), forward};
  }
};

// Slab-method ray/AABB intersection.  Unlike the legacy vrComputeIntersection
// (which returned "miss" whenever t_near < 0, so a camera inside the volume
// saw background), entry clamps to t = 0.
bool intersect_slab(const ray &r, const double omin[3], const double omax[3], double &t0,
                    double &t1) {
  t0 = 0.0;
  t1 = std::numeric_limits<double>::infinity();
  const double org[3] = {r.origin.x, r.origin.y, r.origin.z};
  const double dir[3] = {r.direction.x, r.direction.y, r.direction.z};
  for (int a = 0; a < 3; ++a) {
    if (dir[a] == 0.0) {
      if (org[a] < omin[a] || org[a] > omax[a])
        return false;
      continue;
    }
    double ta = (omin[a] - org[a]) / dir[a];
    double tb = (omax[a] - org[a]) / dir[a];
    if (ta > tb)
      std::swap(ta, tb);
    t0 = std::max(t0, ta);
    t1 = std::min(t1, tb);
  }
  // Finiteness guard: a NaN/degenerate box (e.g. a transform gone wrong)
  // must produce a miss, not an unbounded march.
  return t0 <= t1 && std::isfinite(t0) && std::isfinite(t1);
}

inline bool intersect_box(const ray &r, const cvc::bounding_box &box, double &t0, double &t1) {
  const double omin[3] = {box.minx, box.miny, box.minz};
  const double omax[3] = {box.maxx, box.maxy, box.maxz};
  return intersect_slab(r, omin, omax, t0, t1);
}

inline bool culled_by_planes(const std::vector<cut_plane> &planes, const vec3d &p) {
  for (const cut_plane &c : planes) {
    const vec3d n(c.normal);
    if (dot(p - vec3d(c.point), n) < 0.0)
      return true;
  }
  return false;
}

// Content-generation stamps for the CUDA backend's resident volume cache.
//
// A freshly registered volume gets generation 0, NOT a fresh stamp: registering
// is not a content change, and cvcGL's VolRenNode rebuilds its raycaster's
// volume list on every single frame (worker::run does clear_volumes() +
// add_volume()).  Stamping there would make every frame a cache miss and undo
// the whole point of keeping voxels resident.  Only invalidate_device_volume()
// draws a stamp, from a PROCESS-WIDE counter so that two raycasters sharing a
// voxel buffer cannot collide -- and a mismatch always means "re-upload the
// current host bytes", so a disagreement costs an upload, never staleness.
inline constexpr std::uint64_t initial_content_generation = 0;

std::uint64_t next_content_generation() {
  static std::atomic<std::uint64_t> counter{initial_content_generation};
  return counter.fetch_add(1, std::memory_order_relaxed) + 1;
}

inline unsigned char to_byte(float c) {
  // The inverted test maps NaN to 0 instead of an undefined float->uchar cast.
  const float v = !(c > 0.f) ? 0.f : (c < 1.f ? c : 1.f);
  return static_cast<unsigned char>(v * 255.0f + 0.5f);
}

// FNV-1a over the light pass's inputs, so the built shadow maps can be reused
// across camera motion and rebuilt on anything else.  Scalars are hashed one
// at a time rather than whole structs: struct padding is indeterminate, and a
// key that changed with the padding would rebuild at random.
struct fingerprint {
  std::uint64_t h = 14695981039346656037ull;

  void bytes(const void *p, std::size_t n) {
    const unsigned char *b = static_cast<const unsigned char *>(p);
    for (std::size_t i = 0; i < n; ++i) {
      h ^= b[i];
      h *= 1099511628211ull;
    }
  }
  void add(double v) {
    // Normalize the two zeros so -0.0 and 0.0 do not read as different scenes.
    if (v == 0.0)
      v = 0.0;
    bytes(&v, sizeof(v));
  }
  void add(std::int64_t v) { bytes(&v, sizeof(v)); }
  void add(std::uint64_t v) { bytes(&v, sizeof(v)); }
  void add(int v) { add(std::int64_t(v)); }
  void add(bool v) { add(std::int64_t(v ? 1 : 0)); }
  void add(float v) { add(double(v)); }
  void add(const void *p) { add(std::uint64_t(reinterpret_cast<std::uintptr_t>(p))); }
  void add(const std::array<double, 3> &a) {
    add(a[0]);
    add(a[1]);
    add(a[2]);
  }

  // 0 is reserved for "nothing built", so a hash that lands there is nudged.
  std::uint64_t value() const { return h == 0 ? 1 : h; }
};

// Everything the shadow lookup needs, resolved once per render() and shared
// (const) by every tile -- no per-tile scratch beyond the visibility buffer,
// and no allocation inside the march.
struct shadow_context {
  struct entry {
    const shadow_view *view = nullptr;
    const float *depth = nullptr;
  };
  bool active = false;
  bool two_sided = false;
  // The along-ray quantum of the light-view depth latch.  render() sizes it on
  // the latch MECHANISM -- one march step for an exact isosurface intersection,
  // two cells for a transfer-function latch -- and carries the measurements.
  double latch_quantum = 0.0;
  double bias_scale = 0.0;
  double slope_scale = 0.0;
  float strength = 1.f;
  std::vector<entry> maps;
  // Light index -> index into `maps`, or -1 for a light that never casts.
  std::vector<int> map_of_light;

  // Fills out[0, nlights) with each light's visibility at world point `p` on a
  // surface whose unit world normal is `normal`.
  void visibility(const vec3d &p, const vec3d &normal, float *out, std::size_t nlights) const {
    for (std::size_t i = 0; i < nlights; ++i)
      out[i] = 1.f;
    if (!active)
      return;
    const std::size_t n = std::min(nlights, map_of_light.size());
    for (std::size_t i = 0; i < n; ++i) {
      const int m = map_of_light[i];
      if (m < 0)
        continue;
      const entry &e = maps[std::size_t(m)];
      const vec3d ldir = -vec3d(e.view->forward); // forward == -light direction
      const double n_dot_l = dot(normal, ldir);
      // A one-sided surface facing away from the light already gets neither
      // diffuse nor specular from it, so there is nothing to attenuate --
      // and skipping the lookup deletes the whole grazing/back-facing acne
      // class at negative cost.
      if (!two_sided && !(n_dot_l > 0.0))
        continue;
      const double bias =
          detail::shadow_bias(*e.view, latch_quantum, bias_scale, slope_scale, n_dot_l);
      out[i] = detail::shadow_visibility(*e.view, e.depth, p, bias, strength);
    }
  }
};

} // namespace

raycaster::raycaster(cvc::app &ctx) : _ctx(ctx) {}

raycaster::~raycaster() = default;

std::size_t raycaster::add_volume(const cvc::volume &vol, volume_settings vs) {
  if (vol.XDim() < 2 || vol.YDim() < 2 || vol.ZDim() < 2)
    throw volren_error("volumes need at least 2 voxels per axis");
  const cvc::bounding_box &b = vol.boundingBox();
  if (!(b.maxx > b.minx) || !(b.maxy > b.miny) || !(b.maxz > b.minz))
    throw volren_error("volume bounding box has zero extent");
  _volumes.push_back(vol); // shallow copy (COW buffer share)
  _volume_settings.push_back(std::move(vs));
  // Recover an invalidation announced BEFORE this (re-)registration.  cvcGL's
  // VolRenNode rebuilds its volume list every time the scene changes, so
  // invalidate_device_volume() followed by clear_volumes()/add_volume() is the
  // ordinary order, not an exotic one -- and resetting to
  // initial_content_generation there would hand the volume back to the device
  // cache with the very stamp its stale block was installed under.
  const auto announced = _announced_generation.find(vol.data_ptr());
  _volume_generation.push_back(
      announced != _announced_generation.end() ? announced->second : initial_content_generation);
  return _volumes.size() - 1;
}

void raycaster::clear_volumes() {
  _volumes.clear();
  _volume_settings.clear();
  _volume_generation.clear();
}

void raycaster::invalidate_device_volume(std::size_t index) {
  if (index >= _volume_generation.size())
    throw volren_error("volume index out of range");
  const std::uint64_t g = next_content_generation();
  _volume_generation[index] = g;
  // Durable so the stamp survives a clear_volumes()/add_volume() cycle.
  _announced_generation[_volumes[index].data_ptr()] = g;
}

void raycaster::invalidate_device_volumes() {
  // ONE generation for the whole call, not one per volume: the device cache is
  // keyed on (device, host pointer, length), so instanced scenes -- nine
  // bunnies sharing one buffer, the case the cache exists for -- would
  // otherwise collide on a single key carrying N distinct stamps, and each
  // volume would evict and re-upload the block the previous one just
  // installed: N uploads and N live device blocks per render instead of one.
  const std::uint64_t g = next_content_generation();
  for (std::size_t i = 0; i < _volume_generation.size(); ++i) {
    _volume_generation[i] = g;
    _announced_generation[_volumes[i].data_ptr()] = g;
  }
}

volume_settings &raycaster::volume_config(std::size_t index) {
  if (index >= _volume_settings.size())
    throw volren_error("volume index out of range");
  return _volume_settings[index];
}

const volume_settings &raycaster::volume_config(std::size_t index) const {
  if (index >= _volume_settings.size())
    throw volren_error("volume index out of range");
  return _volume_settings[index];
}

cvc::bounding_box raycaster::scene_bounds() const {
  if (_volumes.empty())
    throw volren_error("no volumes registered");
  // Manual union: bounding_box::operator+ treats zero-volume boxes as null,
  // and the legacy metavol union was zero-initialized (only correct for
  // origin-straddling volumes) -- neither is wanted here.  Each volume's box
  // enters the union through its model transform (the world-space AABB of
  // the transformed local box).
  bool first = true;
  cvc::bounding_box out;
  for (std::size_t i = 0; i < _volumes.size(); ++i) {
    const cvc::bounding_box &b = _volumes[i].boundingBox();
    const mat4 &mt = _volume_settings[i].model_transform;
    for (int corner = 0; corner < 8; ++corner) {
      const vec3d p =
          mt.transform_point({corner & 1 ? b.maxx : b.minx, corner & 2 ? b.maxy : b.miny,
                              corner & 4 ? b.maxz : b.minz});
      if (first) {
        out.minx = out.maxx = p.x;
        out.miny = out.maxy = p.y;
        out.minz = out.maxz = p.z;
        first = false;
      } else {
        out.minx = std::min(out.minx, p.x);
        out.miny = std::min(out.miny, p.y);
        out.minz = std::min(out.minz, p.z);
        out.maxx = std::max(out.maxx, p.x);
        out.maxy = std::max(out.maxy, p.y);
        out.maxz = std::max(out.maxz, p.z);
      }
    }
  }
  return out;
}

cvc::image raycaster::shadow_map_depth(std::size_t i) const {
  if (i >= _shadow_depth.size())
    throw volren_error("shadow map index out of range");
  return _shadow_depth[i];
}

const shadow_view &raycaster::shadow_map_view(std::size_t i) const {
  if (i >= _shadow_views.size())
    throw volren_error("shadow map index out of range");
  return _shadow_views[i];
}

void raycaster::invalidate_shadow_maps() { _shadow_key = 0; }

void raycaster::ensure_shadow_maps(const cvc::bounding_box &scene) {
  const shadow_settings &sh = _settings.shadows;
  if (!sh.enabled) {
    _shadow_views.clear();
    _shadow_depth.clear();
    _shadow_key = 0;
    return;
  }

  // Which lights cast, in order, deduplicated: a repeated index would pay for
  // a second identical pass and then be shadowed by whichever map won.
  std::vector<int> casters;
  if (sh.lights.empty()) {
    for (std::size_t i = 0; i < _settings.lights.size(); ++i)
      casters.push_back(int(i));
  } else {
    for (const int idx : sh.lights)
      if (std::find(casters.begin(), casters.end(), idx) == casters.end())
        casters.push_back(idx);
  }
  const int resolution =
      std::max(limits::min_shadow_resolution, std::min(sh.resolution, limits::max_raster_dim));

  // ---- cache key ---------------------------------------------------------
  // Everything the light pass reads, and deliberately NOT the camera: the maps
  // are camera-independent, which is what makes shadows free while the user
  // orbits.  `strength`, `bias_scale` and `slope_scale` are consumed by the
  // MAIN march, not by the light pass, so they are absent here on purpose.
  fingerprint fp;
  fp.add(resolution);
  fp.add(sh.min_occluder_opacity);
  fp.add(int(casters.size()));
  for (const int c : casters) {
    fp.add(c);
    fp.add(_settings.lights[std::size_t(c)].direction);
  }
  fp.add(_settings.steps);
  fp.add(_settings.opacity_cutoff);
  fp.add(_settings.depth_alpha_threshold);
  fp.add(scene.minx);
  fp.add(scene.miny);
  fp.add(scene.minz);
  fp.add(scene.maxx);
  fp.add(scene.maxy);
  fp.add(scene.maxz);
  fp.add(int(_settings.cut_planes.size()));
  for (const cut_plane &c : _settings.cut_planes) {
    fp.add(c.point);
    fp.add(c.normal);
  }
  fp.add(int(_volumes.size()));
  for (std::size_t v = 0; v < _volumes.size(); ++v) {
    const cvc::volume &vol = _volumes[v];
    const volume_settings &vs = _volume_settings[v];
    // The voxel BUFFER identity, not its contents: an in-place write through
    // voxels::data_ptr() keeps the pointer and is invisible here, which is
    // exactly what invalidate_shadow_maps() exists to announce.  Every write
    // through the supported API copy-on-writes to a new buffer and lands on a
    // different key.
    fp.add(static_cast<const void *>(*vol));
    fp.add(_volume_generation[v]);
    fp.add(std::int64_t(vol.XDim()));
    fp.add(std::int64_t(vol.YDim()));
    fp.add(std::int64_t(vol.ZDim()));
    fp.add(int(vol.voxelType()));
    fp.add(vol.XMin());
    fp.add(vol.YMin());
    fp.add(vol.ZMin());
    fp.add(vol.XSpan());
    fp.add(vol.YSpan());
    fp.add(vol.ZSpan());
    for (const double m : vs.model_transform.m)
      fp.add(m);
    fp.add(vs.shaded);
    fp.add(vs.unshaded);
    fp.add(vs.window_enabled);
    fp.add(vs.window_min);
    fp.add(vs.window_max);
    fp.add(vs.gradient_ramp.enabled);
    fp.add(vs.gradient_ramp.ramp0);
    fp.add(vs.gradient_ramp.ramp1);
    fp.add(vs.gradient_ramp.ramp2);
    fp.add(vs.gradient_ramp.plateau);
    fp.add(vs.tf_auto_domain);
    // The bake domain, which decides what alpha a sample gets and therefore
    // where the depth latch fires.  min()/max() are cached on the volume and
    // were already forced by the volume prep above for exactly these volumes.
    if ((vs.shaded || vs.unshaded) && vs.tf_auto_domain) {
      fp.add(vol.min());
      fp.add(vol.max());
    }
    fp.add(int(vs.tf.points().size()));
    for (const transfer_point &p : vs.tf.points()) {
      fp.add(p.value);
      fp.add(p.r);
      fp.add(p.g);
      fp.add(p.b);
      fp.add(p.a);
    }
    fp.add(int(vs.isosurfaces.size()));
    for (const isosurface &s : vs.isosurfaces) {
      fp.add(s.value);
      fp.add(s.opacity); // decides whether it casts at all
    }
  }

  const std::uint64_t key = fp.value();
  if (key == _shadow_key)
    return;

  std::vector<shadow_view> views;
  std::vector<cvc::image> depths;
  for (const int li : casters) {
    camera light_cam;
    shadow_view sv;
    // A degenerate light direction simply casts no shadow -- consistent with
    // blinn_phong, where it already contributes nothing (ndotl == 0).
    if (!detail::fit_light_camera(_settings.lights[std::size_t(li)], scene, resolution, light_cam,
                                  sv))
      continue;
    sv.light_index = li;

    raycaster light_rc(_ctx);
    for (std::size_t v = 0; v < _volumes.size(); ++v) {
      volume_settings cs = _volume_settings[v];
      // Only surfaces opaque enough to be believable occluders cast: the depth
      // latch fires on the FIRST isosurface hit whatever that surface's
      // opacity, so an unfiltered decorative shell (volren_bunny --shell, at
      // opacity 0.16 and four world units out) would become the occluder and
      // drop the entire body it wraps into its own shadow -- an offset no
      // bias can rescue.  Transfer-function volumes still cast through the
      // alpha latch and are not filtered.
      cs.isosurfaces.erase(std::remove_if(cs.isosurfaces.begin(), cs.isosurfaces.end(),
                                          [&](const isosurface &s) {
                                            return !(s.opacity >= sh.min_occluder_opacity);
                                          }),
                           cs.isosurfaces.end());
      light_rc.add_volume(_volumes[v], std::move(cs));
    }
    // Inherit the content stamps, or a volume the owner announced as changed
    // would be served to the light pass out of the resident device cache.
    light_rc._volume_generation = _volume_generation;

    render_settings ls = _settings;
    ls.shadows = shadow_settings(); // the light pass casts no shadows of its own
    // The depth latch is driven by accumulated ALPHA only, so shading the
    // light pass would change nothing and cost a Blinn-Phong evaluation per
    // contribution.  Clearing the lights makes that explicit.
    ls.lights.clear();
    // A depth map has no color to filter, and the supersampled resolve keeps
    // the NEAREST sub-sample depth -- which would dilate every occluder by
    // half a pixel of its silhouette.  One ray per texel center.
    ls.supersample = 1;
    light_rc.settings() = std::move(ls);
    light_rc.view() = light_cam;
    // NEVER backend::cuda (strict), even when the parent asked for it: the
    // light pass is an INTERNAL pass and can fall outside the device scope for
    // reasons the caller cannot control, and failing the whole frame because
    // of that would be surprising.  A CPU-built map consumed by a CUDA main
    // pass is the same data either way.
    light_rc.set_backend(_backend == backend::cpu ? backend::cpu : backend::automatic);
    if (_settings.threads != 1) {
      // Safe to share: the light pass runs to completion before the main march
      // fans out, so there is never a second in-flight parallel_for.
      if (!_pool && !_own_pool)
        _own_pool = std::make_unique<cvc::thread_pool>();
      light_rc.set_thread_pool(_pool ? _pool : _own_pool.get());
    }

    frame lf = light_rc.render();
    views.push_back(sv);
    depths.push_back(lf.depth);
  }

  _shadow_views = std::move(views);
  _shadow_depth = std::move(depths);
  _shadow_key = key;
}

frame raycaster::render() {
  if (_volumes.empty())
    throw volren_error("no volumes registered");
  if (_settings.steps < 1)
    throw volren_error("render_settings::steps must be >= 1");
  if (!(_settings.opacity_cutoff > 0.f) || _settings.opacity_cutoff > 1.f)
    throw volren_error("render_settings::opacity_cutoff must be in (0, 1]");
  if (_settings.supersample < 1 || _settings.supersample > limits::max_supersample)
    throw volren_error("render_settings::supersample must be in [1, limits::max_supersample]");
  if (_settings.shadows.enabled) {
    const shadow_settings &sh = _settings.shadows;
    std::size_t casters = _settings.lights.size(); // empty list == every light casts
    if (!sh.lights.empty()) {
      std::vector<int> unique;
      for (const int idx : sh.lights) {
        if (idx < 0 || std::size_t(idx) >= _settings.lights.size())
          throw volren_error("render_settings::shadows.lights holds an index with no light");
        if (std::find(unique.begin(), unique.end(), idx) == unique.end())
          unique.push_back(idx);
      }
      casters = unique.size();
    }
    // Each caster is a full extra render pass, so the cap is refused loudly
    // rather than silently dropping a light.
    if (casters > std::size_t(limits::max_shadow_maps))
      throw volren_error("more shadow-casting lights than limits::max_shadow_maps");
    if (!(sh.strength >= 0.f) || sh.strength > 1.f)
      throw volren_error("render_settings::shadows.strength must be in [0, 1]");
  }

  cvc::app::thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

  const ray_generator rays(_camera);
  const int width = _camera.width;
  const int height = _camera.height;

  // Resolve volumes once: raw buffer + pin, geometry, baked TF.
  std::vector<prepared_volume> prep(_volumes.size());
  for (std::size_t i = 0; i < _volumes.size(); ++i) {
    const cvc::volume &vol = _volumes[i];
    prepared_volume &p = prep[i];
    p.pin = vol.active_storage();
    p.grid.data = *vol;
    p.grid.type = vol.voxelType();
    p.grid.dimx = std::int64_t(vol.XDim());
    p.grid.dimy = std::int64_t(vol.YDim());
    p.grid.dimz = std::int64_t(vol.ZDim());
    p.grid.minb = {vol.XMin(), vol.YMin(), vol.ZMin()};
    p.grid.span = {vol.XSpan(), vol.YSpan(), vol.ZSpan()};
    p.vs = &_volume_settings[i];
    p.any_iso = !p.vs->isosurfaces.empty();
    for (const double m : p.vs->model_transform.m)
      if (!std::isfinite(m))
        throw volren_error("volume model_transform must be finite");
    p.transformed = !p.vs->model_transform.is_identity();
    if (p.transformed)
      p.world_to_local = p.vs->model_transform.affine_inverse();
    p.cull_lo = {p.grid.minb.x - 2.0 * p.grid.span.x, p.grid.minb.y - 2.0 * p.grid.span.y,
                 p.grid.minb.z - 2.0 * p.grid.span.z};
    p.cull_hi = {p.grid.minb.x + p.grid.span.x * double(p.grid.dimx),
                 p.grid.minb.y + p.grid.span.y * double(p.grid.dimy),
                 p.grid.minb.z + p.grid.span.z * double(p.grid.dimz)};
    if (p.vs->shaded || p.vs->unshaded) {
      const double lo = p.vs->tf_auto_domain ? vol.min() : p.vs->tf.domain_min();
      const double hi = p.vs->tf_auto_domain ? vol.max() : p.vs->tf.domain_max();
      p.tf = baked_transfer_function(p.vs->tf, lo, hi);
    }
  }

  const cvc::bounding_box scene = scene_bounds();
  const double diag = std::sqrt((scene.maxx - scene.minx) * (scene.maxx - scene.minx) +
                                (scene.maxy - scene.miny) * (scene.maxy - scene.miny) +
                                (scene.maxz - scene.minz) * (scene.maxz - scene.minz));
  const double unit_step = diag / double(_settings.steps);
  const render_settings &rs = _settings;

  // ---- volumetric shadows -------------------------------------------------
  // Built (or reused) BEFORE the backend split, so both marchers consume the
  // same maps: a shadow map is data, and the parity contract between the two
  // backends is about how that data is consumed, not where it was produced.
  ensure_shadow_maps(scene);

  // ---- the along-ray quantum of the light-view depth latch ----------------
  // It depends on HOW the light pass latched, and the two mechanisms differ by
  // an order of magnitude:
  //
  //  * An ISOSURFACE latch is an exact ray/MC intersection -- not quantized at
  //    all -- so `unit_step` is already generous.  Measured on a self-shadowing
  //    64^3 sphere: worst disagreement 0.42 unit_steps overhead, 0.66 at 30
  //    degrees elevation, against a bias of one.
  //
  //  * A TRANSFER-FUNCTION latch fires on the first march SAMPLE whose
  //    contribution pushed accumulated alpha past the threshold, and the volren
  //    sampling model contributes at most once per CELL.  So the latch walks in
  //    cells, and the receiver SAMPLE the main march shades is cell-quantized
  //    the same way along its own (different) ray -- two independent cells of
  //    slack, one per ray.  Measured on the same sphere with a one-cell-thick
  //    shaded-TF shell: worst 1.3 cell diagonals, and FLAT in world units from
  //    128 to 1024 steps while growing from 4 to 28 unit_steps.  The step is
  //    measurably the wrong unit; two cells is the right one.
  //
  // Deliberately NOT applying the cell quantum to a pure-isosurface scene:
  // that would peter-pan every crease shadow on the renderer's flagship
  // content (an opaque SDF isosurface) by several cells to pay for a latch
  // mechanism it does not use.
  double latch_quantum = unit_step;
  for (const prepared_volume &p : prep) {
    if (!(p.vs->shaded || p.vs->unshaded) || p.tf.empty())
      continue;
    // The cell is a parallelepiped once a model transform is involved; its
    // world diagonal is the linear part applied to the local span.
    const vec3d local_diag(p.grid.span.x, p.grid.span.y, p.grid.span.z);
    const vec3d world_diag =
        p.transformed ? p.vs->model_transform.transform_vector(local_diag) : local_diag;
    latch_quantum = std::max(latch_quantum, 2.0 * length(world_diag));
  }

  shadow_context shadows;
  shadows.active = rs.shadows.enabled && !_shadow_views.empty() && rs.shadows.strength > 0.f;
  if (shadows.active) {
    shadows.two_sided = rs.two_sided_lighting;
    shadows.latch_quantum = latch_quantum;
    shadows.bias_scale = double(rs.shadows.bias_scale);
    shadows.slope_scale = double(rs.shadows.slope_scale);
    shadows.strength = rs.shadows.strength;
    shadows.maps.resize(_shadow_views.size());
    shadows.map_of_light.assign(rs.lights.size(), -1);
    for (std::size_t i = 0; i < _shadow_views.size(); ++i) {
      // A const reference so image::data() does not copy-on-write detach.
      const cvc::image &img = _shadow_depth[i];
      shadows.maps[i].view = &_shadow_views[i];
      shadows.maps[i].depth = reinterpret_cast<const float *>(img.data());
      const int li = _shadow_views[i].light_index;
      if (li >= 0 && std::size_t(li) < shadows.map_of_light.size())
        shadows.map_of_light[std::size_t(li)] = int(i);
    }
  }
  const std::size_t nlights = rs.lights.size();

  // ---- CUDA backend ------------------------------------------------------
  // Opt-in: the default is backend::cpu, so nothing here runs for an existing
  // caller.  The device path covers the scope in raycaster_cuda.h; a scene
  // outside it falls back silently under backend::automatic and throws under
  // backend::cuda, so an explicit request never quietly costs a CPU march.  A
  // device-side failure (cvc::cuda_error) always falls back.
  _backend_used = backend::cpu;
  if (_backend != backend::cpu) {
    const char *reason = nullptr;
    if (!raycast_cuda_available())
      reason = "no usable CUDA device";
    else if (prep.size() > std::size_t(cuda_limits::max_volumes))
      reason = "more volumes than cuda_limits::max_volumes";
    else if (_settings.lights.size() > std::size_t(cuda_limits::max_lights))
      reason = "more lights than cuda_limits::max_lights";
    else if (_settings.cut_planes.size() > std::size_t(cuda_limits::max_cut_planes))
      reason = "more cut planes than cuda_limits::max_cut_planes";
    else
      for (const prepared_volume &p : prep)
        if (p.vs->isosurfaces.size() > std::size_t(cuda_limits::max_isosurfaces)) {
          reason = "more isosurfaces on one volume than cuda_limits::max_isosurfaces";
          break;
        }

    if (reason != nullptr) {
      if (_backend == backend::cuda)
        throw volren_error(std::string("render(): backend::cuda unusable -- ") + reason);
    } else {
      raycast_cuda_request req;
      req.cam = _camera;
      req.volume_count = int(prep.size());

      // The request holds raw pointers into the baked LUTs, so they must
      // outlive the raycast_cuda() call below -- one owner per volume, kept
      // alive by this scope.
      std::vector<std::vector<float>> luts(prep.size());

      for (std::size_t v = 0; v < prep.size(); ++v) {
        const prepared_volume &p = prep[v];
        const volume_settings &vs = *p.vs;
        cuda_volume &cv = req.volumes[v];
        // The buffer stays valid for the call -- and stays identifiable for
        // the resident device cache -- through the pin.
        cv.data = p.grid.data;
        cv.pin = p.pin;
        cv.generation = _volume_generation[v];
        // The resident cache's whole invalidation rule rests on the pin
        // forcing copy-on-write: because the cache co-owns the block, any
        // write through the supported cvc::volume API detaches to a DIFFERENT
        // buffer, so a mutated volume misses by construction.
        //
        // That is true only for HOST-resident volumes.  When using_cuda() is
        // set, active_storage() hands back the CUDA unified block while
        // voxels::preWrite() still tests the separate host shared_array for
        // uniqueness -- our pin is invisible to that test, so a supported
        // write lands IN PLACE and the cache would keep serving pre-mutation
        // voxels.  Stamp a fresh generation every render for such volumes so
        // they are always treated as dirty.
        if (_volumes[v].using_cuda())
          cv.generation = next_content_generation();
        cv.type = p.grid.type;
        cv.dimx = p.grid.dimx;
        cv.dimy = p.grid.dimy;
        cv.dimz = p.grid.dimz;
        cv.minb[0] = p.grid.minb.x;
        cv.minb[1] = p.grid.minb.y;
        cv.minb[2] = p.grid.minb.z;
        cv.span[0] = p.grid.span.x;
        cv.span[1] = p.grid.span.y;
        cv.span[2] = p.grid.span.z;
        cv.transformed = p.transformed;
        for (int i = 0; i < 16; ++i)
          cv.world_to_local[i] = p.world_to_local.m[i];

        // Flatten the baked LUT.  baked_transfer_function exposes no raw view,
        // but entry i IS sample() at the i-th uniform domain value (the
        // nearest-entry lookup round-trips exactly), so the table is
        // reconstructed without touching the public transfer_function header.
        if (!p.tf.empty()) {
          const std::size_t n = p.tf.size();
          const double lo = p.tf.domain_min();
          const double hi = p.tf.domain_max();
          std::vector<float> &lut = luts[v];
          lut.resize(n * 4);
          for (std::size_t i = 0; i < n; ++i) {
            const rgba_f c = p.tf.sample(lo + (hi - lo) * double(i) / double(n - 1));
            lut[i * 4 + 0] = c.r;
            lut[i * 4 + 1] = c.g;
            lut[i * 4 + 2] = c.b;
            lut[i * 4 + 3] = c.a;
          }
          cv.tf_active = true;
          cv.lut = lut.data();
          cv.lut_size = int(n);
          cv.tf_lo = lo;
          cv.tf_hi = hi;
        }

        cv.gradient_ramp_enabled = vs.gradient_ramp.enabled;
        cv.ramp0 = vs.gradient_ramp.ramp0;
        cv.ramp1 = vs.gradient_ramp.ramp1;
        cv.ramp2 = vs.gradient_ramp.ramp2;
        cv.gradient_plateau = vs.gradient_ramp.plateau;

        cv.isosurface_count = int(vs.isosurfaces.size());
        for (std::size_t i = 0; i < vs.isosurfaces.size(); ++i) {
          const isosurface &s = vs.isosurfaces[i];
          cv.isosurfaces[i].value = s.value;
          cv.isosurfaces[i].opacity = s.opacity;
          cv.isosurfaces[i].color[0] = s.color[0];
          cv.isosurfaces[i].color[1] = s.color[1];
          cv.isosurfaces[i].color[2] = s.color[2];
          cv.isosurfaces[i].shininess = s.shininess;
        }

        cv.shaded = vs.shaded;
        cv.unshaded = vs.unshaded;
        cv.window_enabled = vs.window_enabled;
        cv.window_min = vs.window_min;
        cv.window_max = vs.window_max;
      }

      req.light_count = int(rs.lights.size());
      for (std::size_t i = 0; i < rs.lights.size(); ++i) {
        for (int c = 0; c < 3; ++c) {
          req.lights[i].color[c] = rs.lights[i].color[c];
          req.lights[i].direction[c] = rs.lights[i].direction[c];
        }
      }
      req.cut_plane_count = int(rs.cut_planes.size());
      for (std::size_t i = 0; i < rs.cut_planes.size(); ++i) {
        for (int c = 0; c < 3; ++c) {
          req.cut_planes[i].point[c] = rs.cut_planes[i].point[c];
          req.cut_planes[i].normal[c] = rs.cut_planes[i].normal[c];
        }
      }

      req.scene_min[0] = scene.minx;
      req.scene_min[1] = scene.miny;
      req.scene_min[2] = scene.minz;
      req.scene_max[0] = scene.maxx;
      req.scene_max[1] = scene.maxy;
      req.scene_max[2] = scene.maxz;
      req.steps = rs.steps;
      req.unit_step = unit_step;
      req.opacity_cutoff = rs.opacity_cutoff;
      req.depth_alpha_threshold = rs.depth_alpha_threshold;
      req.ambient = rs.ambient;
      req.two_sided = rs.two_sided_lighting;
      req.background[0] = rs.background[0];
      req.background[1] = rs.background[1];
      req.background[2] = rs.background[2];
      req.tf_shininess = defaults::shininess;
      req.supersample = rs.supersample;

      // The maps ride as HOST pointers into _shadow_depth, which outlives the
      // call; raycast_cuda stages them into one device allocation like `lut`.
      if (shadows.active) {
        req.shadow_map_count = int(shadows.maps.size());
        for (std::size_t i = 0; i < shadows.maps.size(); ++i) {
          const shadow_view &sv = *shadows.maps[i].view;
          cuda_shadow_map &cm = req.shadow_maps[i];
          for (int c = 0; c < 3; ++c) {
            cm.eye[c] = sv.eye[c];
            cm.right[c] = sv.right[c];
            cm.up[c] = sv.up[c];
            cm.forward[c] = sv.forward[c];
          }
          cm.parallel_scale = sv.parallel_scale;
          cm.texel_world = sv.texel_world;
          cm.width = sv.width;
          cm.height = sv.height;
          cm.light_index = sv.light_index;
          cm.depth = shadows.maps[i].depth;
        }
        req.shadow_strength = shadows.strength;
        // The constant term is scene-scaled, not view-scaled, so it is folded
        // host-side and the kernel only adds the per-sample slope term.
        req.shadow_bias_constant = shadows.bias_scale * shadows.latch_quantum;
        req.shadow_slope_scale = shadows.slope_scale;
      }

      try {
        frame gpu = raycast_cuda(req);
        _backend_used = backend::cuda;
        _ctx.threadProgress(1.0);
        return gpu;
      } catch (const cvc::cuda_error &) {
        // A device-side failure is recoverable: fall through to the CPU march
        // (voxels.cpp uses the same catch-and-degrade contract).
        if (_backend == backend::cuda)
          throw;
        _backend_used = backend::cpu;
      } catch (const cvc::volren_error &) {
        // raycast_cuda validates the request itself and can reject a scene the
        // host-side scope check above accepted.  backend::automatic promises a
        // SILENT fallback, so only an explicit backend::cuda propagates.
        if (_backend == backend::cuda)
          throw;
        _backend_used = backend::cpu;
      }
    }
  }

  frame out;
  out.color = cvc::image(width, height, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  out.depth = cvc::image(width, height, cvc::image::pixel_format::GRAY, cvc::image::data_type::f32);
  unsigned char *color_px = out.color.data();
  float *depth_px = reinterpret_cast<float *>(out.depth.data());

  const std::size_t nvol = prep.size();

  // Per-tile scratch, reused across the tile's rays: the spline-gradient
  // neighborhood cache (valid across rays -- it is keyed on the cell index)
  // and the per-ray last-cell tracker.
  struct iso_hit {
    double t;
    std::array<float, 3> color;
    float opacity;
  };
  // One volume this ray can actually reach, with the world-t window in which
  // it can contribute.  Built once per ray so the march loop never pays for a
  // volume the ray misses entirely -- see prepared_volume::cull_lo/cull_hi.
  struct active_volume {
    std::size_t index;
    double t_lo, t_hi; // clipped to the scene span, padded by one step
    vec3d lorg, ldir;  // the ray in this volume's local frame (t is preserved)
  };
  struct tile_scratch {
    std::vector<detail::spline_gradient_cache> spline;
    std::vector<std::array<std::int64_t, 3>> last_cell;
    std::vector<iso_hit> hits;
    std::vector<active_volume> active;
    // Per-light shadow visibility for the sample being shaded.  Sized once
    // per tile rather than kept as a fixed array so the CPU path's unbounded
    // light count survives; the shading sites never allocate.
    std::vector<float> vis;
  };
  // What one ray leaves behind: the ASSOCIATED (premultiplied) color it
  // accumulated, its opacity, and its latched eye-space depth.  The background
  // over-blend is deliberately NOT applied here -- see the resolve below.
  struct ray_result {
    float r, g, b, a;
    float depth;
  };

  const auto march_ray = [&](const ray &r, tile_scratch &scratch, ray_result &out) {
    out.r = out.g = out.b = out.a = 0.f;
    out.depth = std::numeric_limits<float>::infinity();

    double t0 = 0.0, t1 = 0.0;
    const bool hit_scene = intersect_box(r, scene, t0, t1);

    // ---- Per-ray volume culling -------------------------------------------
    // Hoisted out of the march: each volume's [t_enter, t_exit] against ITS
    // box (through the model transform, exactly as the isosurface DDA already
    // does) is solved once, so a step visits only the volumes whose window
    // contains t instead of calling cell_index() on every volume at every
    // step and usually missing.  Pure work elimination: the cull box strictly
    // contains cell_index()'s acceptance region and the window is padded by a
    // step, so no sample that used to contribute can be skipped.
    scratch.active.clear();
    if (hit_scene) {
      for (std::size_t v = 0; v < nvol; ++v) {
        const prepared_volume &p = prep[v];
        active_volume a;
        a.index = v;
        a.lorg = p.to_local_point(r.origin);
        a.ldir = p.to_local_vector(r.direction); // unnormalized: t preserved
        const double lo[3] = {p.cull_lo.x, p.cull_lo.y, p.cull_lo.z};
        const double hi[3] = {p.cull_hi.x, p.cull_hi.y, p.cull_hi.z};
        double tv0 = 0.0, tv1 = 0.0;
        if (!intersect_slab(ray{a.lorg, a.ldir}, lo, hi, tv0, tv1))
          continue;
        tv0 = std::max(tv0, t0);
        tv1 = std::min(tv1, t1);
        if (!(tv0 <= tv1))
          continue;
        a.t_lo = tv0 - unit_step;
        a.t_hi = tv1 + unit_step;
        scratch.active.push_back(a);
      }
    }

    // A ray that reaches no volume composites nothing, so the loop below would
    // walk every step to produce exactly this: zero accumulation, depth +inf
    // (the resolve then over-blends the full residual transparency with the
    // background, exactly as it does for a ray that marched and hit nothing).
    if (scratch.active.empty())
      return;

    const vec3d view_vec = -r.direction; // toward the viewer, unit
    const double z_scale = dot(r.direction, rays.forward);

    float acc_r = 0.f, acc_g = 0.f, acc_b = 0.f, acc_a = 0.f;
    bool depth_set = false;

    // Only the reachable volumes need resetting -- the rest are never visited
    // by this ray, so their tracker is never read.
    for (const active_volume &a : scratch.active)
      scratch.last_cell[a.index] = {-1, -1, -1};

    // Front-to-back associated-color compositing; also latches the depth map
    // the first time accumulated alpha crosses the threshold.
    const auto composite = [&](const std::array<float, 3> &c, float a, double t) {
      const float ratio = a * (1.f - acc_a);
      acc_r += c[0] * ratio;
      acc_g += c[1] * ratio;
      acc_b += c[2] * ratio;
      acc_a += ratio;
      if (!depth_set && acc_a >= rs.depth_alpha_threshold) {
        out.depth = float(t * z_scale);
        depth_set = true;
      }
    };

    // ---- Isosurface hits: exact per-cell ray traversal ---------------------
    // The legacy tracer tested a cell only when a march SAMPLE landed in it,
    // so cells the ray merely clipped (traversal shorter than one step) were
    // skipped -- the famous black-speckle artifact, worse at finer grids.
    // Here every cell the ray actually crosses is enumerated with an
    // Amanatides-Woo DDA and MC-intersected exactly; the hits are then merged
    // into the compositing stream at their ray parameter, preserving the
    // front-to-back interleave with the transfer-function samples.
    scratch.hits.clear();
    for (const active_volume &av : scratch.active) {
      const std::size_t v = av.index;
      const prepared_volume &p = prep[v];
      const volume_settings &vs = *p.vs;
      if (!p.any_iso)
        continue;

      // The local ray was solved for the cull test; vbox below sits a full
      // voxel inside the cull box, so a culled volume could not have produced
      // a DDA cell either.
      const vec3d lorg = av.lorg;
      const vec3d ldir = av.ldir; // unnormalized: t preserved
      const ray local_ray{lorg, ldir};

      // Clip the world-t range to this volume's local box.
      const cvc::bounding_box vbox(p.grid.minb.x, p.grid.minb.y, p.grid.minb.z,
                                   p.grid.minb.x + p.grid.span.x * double(p.grid.dimx - 1),
                                   p.grid.minb.y + p.grid.span.y * double(p.grid.dimy - 1),
                                   p.grid.minb.z + p.grid.span.z * double(p.grid.dimz - 1));
      double tv0 = 0.0, tv1 = 0.0;
      if (!intersect_box(local_ray, vbox, tv0, tv1))
        continue;
      tv0 = std::max(tv0, t0);
      tv1 = std::min(tv1, t1);
      if (!(tv0 <= tv1))
        continue;

      // Start half a hair inside so the entry cell resolves.
      const double t_start = tv0 + (tv1 - tv0) * 1e-9;
      std::int64_t idx[3];
      if (!p.grid.cell_index(lorg + ldir * t_start, idx))
        continue;

      const double ld[3] = {ldir.x, ldir.y, ldir.z};
      const double lmin[3] = {p.grid.minb.x, p.grid.minb.y, p.grid.minb.z};
      const double lspan[3] = {p.grid.span.x, p.grid.span.y, p.grid.span.z};
      const std::int64_t dims[3] = {p.grid.dimx, p.grid.dimy, p.grid.dimz};
      const double lorg_a[3] = {lorg.x, lorg.y, lorg.z};
      std::int64_t stepc[3];
      double t_max[3], t_delta[3];
      for (int a = 0; a < 3; ++a) {
        if (ld[a] > 0.0) {
          stepc[a] = 1;
          t_delta[a] = lspan[a] / ld[a];
          t_max[a] = ((lmin[a] + double(idx[a] + 1) * lspan[a]) - lorg_a[a]) / ld[a];
        } else if (ld[a] < 0.0) {
          stepc[a] = -1;
          t_delta[a] = -lspan[a] / ld[a];
          t_max[a] = ((lmin[a] + double(idx[a]) * lspan[a]) - lorg_a[a]) / ld[a];
        } else {
          stepc[a] = 0;
          t_delta[a] = std::numeric_limits<double>::infinity();
          t_max[a] = std::numeric_limits<double>::infinity();
        }
      }

      const std::int64_t max_cells = dims[0] + dims[1] + dims[2] + 3;
      double t_cell = tv0;
      for (std::int64_t n = 0; n < max_cells && t_cell <= tv1; ++n) {
        float vals[8];
        p.grid.corners(idx[0], idx[1], idx[2], vals);
        float min_val = vals[0], max_val = vals[0];
        for (int j = 1; j < 8; ++j) {
          min_val = std::min(min_val, vals[j]);
          max_val = std::max(max_val, vals[j]);
        }

        detail::mc_cell cell;
        bool cell_filled = false;
        for (const isosurface &surf : vs.isosurfaces) {
          if (surf.value < min_val || surf.value > max_val)
            continue;
          if (!cell_filled) {
            cell.id[0] = idx[0];
            cell.id[1] = idx[1];
            cell.id[2] = idx[2];
            cell.orig = p.grid.minb;
            cell.span = p.grid.span;
            cell.from_binary_corners(vals);
            cell_filled = true;
          }
          float w[3];
          double t_hit = 0.0;
          if (!detail::intersect_isosurface_in_cell(local_ray, float(surf.value), cell, w, t_hit))
            continue;
          if (t_hit < t0 || t_hit > tv1 + unit_step)
            continue;
          // One world hit point, used by the cut-plane test and the shadow
          // lookup alike (six flops once per accepted MC intersection).
          const vec3d hit_p = r.origin + r.direction * t_hit;
          if (!rs.cut_planes.empty() && culled_by_planes(rs.cut_planes, hit_p))
            continue;
          const vec3d grad = scratch.spline[v].evaluate(p.grid, idx, w);
          const vec3d normal = normalized(p.normal_to_world(grad));
          // The hit is shaded here, at collection time, and visibility depends
          // only on (p, N) -- never on accumulated alpha -- so the lookup
          // belongs here and composite_hits_up_to stays untouched.
          const float *vis = nullptr;
          if (shadows.active) {
            shadows.visibility(hit_p, normal, scratch.vis.data(), nlights);
            vis = scratch.vis.data();
          }
          const std::array<float, 3> shaded =
              detail::blinn_phong(surf.color, normal, view_vec, rs.lights, rs.two_sided_lighting,
                                  rs.ambient, surf.shininess, vis);
          scratch.hits.push_back({t_hit, shaded, surf.opacity});
        }

        // Advance to the neighboring cell across the nearest boundary.
        int axis = 0;
        if (t_max[1] < t_max[axis])
          axis = 1;
        if (t_max[2] < t_max[axis])
          axis = 2;
        t_cell = t_max[axis];
        idx[axis] += stepc[axis];
        if (idx[axis] < 0 || idx[axis] > dims[axis] - 2)
          break;
        t_max[axis] += t_delta[axis];
      }
    }
    std::stable_sort(scratch.hits.begin(), scratch.hits.end(),
                     [](const iso_hit &a, const iso_hit &b) { return a.t < b.t; });
    std::size_t hit_cursor = 0;

    const auto composite_hits_up_to = [&](double t_limit) {
      while (hit_cursor < scratch.hits.size() && scratch.hits[hit_cursor].t <= t_limit &&
             acc_a < rs.opacity_cutoff) {
        const iso_hit &h = scratch.hits[hit_cursor++];
        // An isosurface hit latches the depth map on its own -- the frame
        // contract is "first iso hit or first threshold-crossing sample,
        // whichever comes first".
        if (!depth_set) {
          out.depth = float(h.t * z_scale);
          depth_set = true;
        }
        composite(h.color, h.opacity, h.t);
      }
    };

    for (double t = t0; t <= t1 && acc_a < rs.opacity_cutoff; t += unit_step) {
      composite_hits_up_to(t);
      const vec3d pnt = r.origin + r.direction * t;
      if (!rs.cut_planes.empty() && culled_by_planes(rs.cut_planes, pnt))
        continue;

      for (const active_volume &av : scratch.active) {
        // Skip the volumes whose slab window this step is outside of: the
        // to_local_point + cell_index below could only miss there.
        if (t < av.t_lo || t > av.t_hi)
          continue;
        const std::size_t v = av.index;
        const prepared_volume &p = prep[v];
        const volume_settings &vs = *p.vs;

        // Sample in volume-local space (the scene-graph model transform).
        const vec3d lpnt = p.to_local_point(pnt);
        std::int64_t idx[3];
        if (!p.grid.cell_index(lpnt, idx))
          continue;
        std::array<std::int64_t, 3> &last = scratch.last_cell[v];
        if (idx[0] == last[0] && idx[1] == last[1] && idx[2] == last[2])
          continue; // one contribution per cell (the volren sampling model)
        last = {idx[0], idx[1], idx[2]};

        float vals[8];
        float min_val = 0.f, max_val = 0.f;
        bool have_corners = false;
        const auto fetch_corners = [&] {
          if (have_corners)
            return;
          p.grid.corners(idx[0], idx[1], idx[2], vals);
          min_val = max_val = vals[0];
          for (int j = 1; j < 8; ++j) {
            min_val = std::min(min_val, vals[j]);
            max_val = std::max(max_val, vals[j]);
          }
          have_corners = true;
        };

        // (Isosurface hits were collected exactly by the DDA above and are
        // merged in by composite_hits_up_to.)

        // 2. Unshaded transfer function (legacy COL_DENSITY).
        if (vs.unshaded) {
          fetch_corners();
          const bool in_window = !vs.window_enabled || (double(min_val) <= vs.window_max &&
                                                        double(max_val) >= vs.window_min);
          if (in_window) {
            float w[3];
            p.grid.local_weights(lpnt, idx[0], idx[1], idx[2], w);
            const float den = detail::trilinear(w, vals);
            const rgba_f s = p.tf.sample(den);
            if (s.a > 0.f)
              composite({s.r, s.g, s.b}, s.a, t);
          }
        }

        // 3. Shaded transfer function (legacy RAY_CASTING).
        if (vs.shaded) {
          fetch_corners();
          float w[3];
          p.grid.local_weights(lpnt, idx[0], idx[1], idx[2], w);
          const float den = detail::trilinear(w, vals);
          if (vs.window_enabled && (double(den) < vs.window_min || double(den) > vs.window_max))
            continue;
          const vec3d grad = scratch.spline[v].evaluate(p.grid, idx, w);
          const rgba_f s = p.tf.sample(den);
          const float a = s.a * vs.gradient_ramp.factor(length(grad));
          if (a > 0.f) {
            const vec3d normal = normalized(p.normal_to_world(grad));
            const float *vis = nullptr;
            if (shadows.active) {
              shadows.visibility(pnt, normal, scratch.vis.data(), nlights);
              vis = scratch.vis.data();
            }
            const std::array<float, 3> shaded =
                detail::blinn_phong({s.r, s.g, s.b}, normal, view_vec, rs.lights,
                                    rs.two_sided_lighting, rs.ambient, defaults::shininess, vis);
            composite(shaded, a, t);
          }
        }
      }
    }
    // Hits between the last sample and the exit point.
    composite_hits_up_to(t1);

    out.r = acc_r;
    out.g = acc_g;
    out.b = acc_b;
    out.a = acc_a;
  };

  // ---- Supersampled resolve ------------------------------------------------
  // `supersample` sub-samples per pixel EDGE on a REGULAR grid: offsets
  // ((i+0.5)/n, (j+0.5)/n), n^2 rays, no jitter.  Determinism is a documented
  // contract of this renderer (byte-identical across runs and thread counts),
  // and a fixed grid also makes the placement checkable: an n-supersampled
  // W x W render lands its rays on exactly the pixel centers of an
  // (n*W) x (n*W) single-sampled one, which is what pins the offsets in the
  // tests.  A rotated grid would resolve near-axis-aligned edges into more
  // levels for the same ray count, but it is not expressible as "the raster
  // you would have rendered", so the identity above -- the only cheap exact
  // check on sub-pixel placement -- would be gone.
  //
  // COLOR resolve, and why it is a plain unweighted mean of the STRAIGHT RGBA:
  // frame::color's RGB channel is the color you display -- the volume's
  // associated color already over-blended with the background -- so it is NOT
  // premultiplied by the alpha stored beside it (a missed ray carries the full
  // background color at alpha 0).  Multiplying it by alpha before averaging,
  // the reflex when a resolve is described as "premultiplied", would erase the
  // background contribution of every partially covered pixel: a pixel half
  // covered by an opaque red surface over a white background would resolve to
  // half red and half BLACK instead of half red and half white -- a dark fringe
  // around every silhouette on any non-black background.  Averaging the
  // resolved straight values is also exactly right rather than merely safe:
  // the over-blend `rgb + background * (1 - a)` is affine in (rgb, a), so a box
  // filter commutes with it -- resolving each sub-sample and averaging equals
  // averaging the associated colors and over-blending once.
  //
  // DEPTH resolve: the NEAREST finite sub-sample depth, never an average.  The
  // depth map exists to depth-test volume pixels against opaque scene geometry
  // per pixel (cvcGL's VolRenNode converts it to window z).  Averaging across a
  // silhouette invents a surface at a depth NO sub-sample saw -- halfway
  // between the foreground and the +inf background, i.e. behind the object --
  // so geometry passing between them would punch through the edge.  The
  // nearest hit is a depth some sub-sample actually measured, and it is the
  // conservative one: a pixel with any foreground coverage occludes at the
  // foreground's depth.  min is also order-independent, so the resolve cannot
  // become a determinism hazard.
  const int ss = rs.supersample;
  // Reciprocal taken in DOUBLE so the CUDA mirror can match it exactly:
  // raycast.cu is compiled with --use_fast_math, which routes float division
  // to an approximate reciprocal but leaves double division correctly rounded.
  const float inv_samples = float(1.0 / double(ss * ss));

  const auto render_ray = [&](int px, int py, tile_scratch &scratch) {
    unsigned char *cpx = color_px + (std::size_t(py) * width + px) * 4;
    float *dpx = depth_px + (std::size_t(py) * width + px);

    float sum_r = 0.f, sum_g = 0.f, sum_b = 0.f, sum_a = 0.f;
    float nearest = std::numeric_limits<float>::infinity();

    for (int sj = 0; sj < ss; ++sj)
      for (int si = 0; si < ss; ++si) {
        ray_result rr;
        march_ray(rays.at(px, py, (double(si) + 0.5) / double(ss), (double(sj) + 0.5) / double(ss)),
                  scratch, rr);
        // Over-blend the remaining transparency with the background (replaces
        // the legacy divide-by-alpha normalization of saturated rays), then
        // accumulate the resolved straight RGBA.
        const float rest = 1.f - rr.a;
        sum_r += rr.r + rs.background[0] * rest;
        sum_g += rr.g + rs.background[1] * rest;
        sum_b += rr.b + rs.background[2] * rest;
        sum_a += rr.a;
        if (rr.depth < nearest)
          nearest = rr.depth;
      }

    // ss == 1 scales by an exact 1.0f and mins one value against +inf, both
    // exact, so the single-sample path is bit-identical to the
    // pre-supersampling one.
    cpx[0] = to_byte(sum_r * inv_samples);
    cpx[1] = to_byte(sum_g * inv_samples);
    cpx[2] = to_byte(sum_b * inv_samples);
    cpx[3] = to_byte(sum_a * inv_samples);
    *dpx = nearest;
  };

  // Tile decomposition: disjoint pixel regions, so any scheduling order and
  // thread count produce byte-identical output.
  const int tile = defaults::tile_size;
  const int tiles_x = (width + tile - 1) / tile;
  const int tiles_y = (height + tile - 1) / tile;
  const int ntiles = tiles_x * tiles_y;

  const auto render_tile = [&](int tid, tile_scratch &scratch) {
    const int tx = (tid % tiles_x) * tile;
    const int ty = (tid / tiles_x) * tile;
    const int xend = std::min(tx + tile, width);
    const int yend = std::min(ty + tile, height);
    for (int py = ty; py < yend; ++py)
      for (int px = tx; px < xend; ++px)
        render_ray(px, py, scratch);
  };

  const auto make_scratch = [&] {
    tile_scratch s;
    s.spline.resize(nvol);
    s.last_cell.resize(nvol);
    s.active.reserve(nvol);
    s.vis.assign(nlights, 1.f);
    return s;
  };

  if (_settings.threads == 1) {
    tile_scratch scratch = make_scratch();
    for (int tid = 0; tid < ntiles; ++tid) {
      boost::this_thread::interruption_point();
      render_tile(tid, scratch);
      _ctx.threadProgress(double(tid + 1) / double(ntiles));
    }
  } else {
    // Default to a raycaster-owned pool: cvc::thread_pool supports only one
    // in-flight parallel_for, so borrowing the app-wide computePool() could
    // deadlock against another subsystem's concurrent fan-out.
    if (!_pool && !_own_pool)
      _own_pool = std::make_unique<cvc::thread_pool>();
    cvc::thread_pool &pool = _pool ? *_pool : *_own_pool;
    const int max_par = int(_settings.threads); // 0 => whole pool
    // Slabs keep interruption + progress responsive without throwing from
    // inside pool tasks.  A throwing tile (e.g. bad_alloc in make_scratch)
    // records the first exception and turns the remaining tiles into no-ops;
    // the orchestrator rethrows it -- an exception must never escape a pool
    // worker (std::terminate) or unwind a live fan-out.
    std::atomic<bool> failed{false};
    std::exception_ptr first_error;
    std::mutex error_mutex;
    const int slab = std::max(1, int(pool.concurrency()) * 4);
    for (int base = 0; base < ntiles && !failed.load(std::memory_order_relaxed); base += slab) {
      boost::this_thread::interruption_point();
      const int count = std::min(slab, ntiles - base);
      pool.parallel_for(
          count,
          [&](int i) {
            if (failed.load(std::memory_order_relaxed))
              return;
            try {
              tile_scratch scratch = make_scratch();
              render_tile(base + i, scratch);
            } catch (...) {
              std::lock_guard<std::mutex> lock(error_mutex);
              if (!first_error)
                first_error = std::current_exception();
              failed.store(true, std::memory_order_relaxed);
            }
          },
          max_par);
      _ctx.threadProgress(double(std::min(base + slab, ntiles)) / double(ntiles));
    }
    if (first_error)
      std::rethrow_exception(first_error);
  }
  _ctx.threadProgress(1.0);

  return out;
}

} // namespace volren
} // namespace cvc
