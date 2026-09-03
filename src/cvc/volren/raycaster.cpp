#include <algorithm>
#include <atomic>
#include <boost/current_function.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/thread/thread.hpp>
#include <cmath>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/thread_pool.h>
#include <cvc/volren/detail/cell_intersect.h>
#include <cvc/volren/detail/sampler.h>
#include <cvc/volren/detail/shading.h>
#include <cvc/volren/detail/spline_gradient.h>
#include <cvc/volren/raycaster.h>
#include <limits>
#include <memory>
#include <mutex>

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

  ray at(int px, int py) const {
    const double u = (double(px) + 0.5) / double(width) * 2.0 - 1.0;
    const double v = 1.0 - (double(py) + 0.5) / double(height) * 2.0;
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
bool intersect_box(const ray &r, const cvc::bounding_box &box, double &t0, double &t1) {
  t0 = 0.0;
  t1 = std::numeric_limits<double>::infinity();
  const double omin[3] = {box.minx, box.miny, box.minz};
  const double omax[3] = {box.maxx, box.maxy, box.maxz};
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

inline bool culled_by_planes(const std::vector<cut_plane> &planes, const vec3d &p) {
  for (const cut_plane &c : planes) {
    const vec3d n(c.normal);
    if (dot(p - vec3d(c.point), n) < 0.0)
      return true;
  }
  return false;
}

inline unsigned char to_byte(float c) {
  // The inverted test maps NaN to 0 instead of an undefined float->uchar cast.
  const float v = !(c > 0.f) ? 0.f : (c < 1.f ? c : 1.f);
  return static_cast<unsigned char>(v * 255.0f + 0.5f);
}

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
  return _volumes.size() - 1;
}

void raycaster::clear_volumes() {
  _volumes.clear();
  _volume_settings.clear();
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

frame raycaster::render() {
  if (_volumes.empty())
    throw volren_error("no volumes registered");
  if (_settings.steps < 1)
    throw volren_error("render_settings::steps must be >= 1");
  if (!(_settings.opacity_cutoff > 0.f) || _settings.opacity_cutoff > 1.f)
    throw volren_error("render_settings::opacity_cutoff must be in (0, 1]");

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

  frame out;
  out.color = cvc::image(width, height, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  out.depth = cvc::image(width, height, cvc::image::pixel_format::GRAY, cvc::image::data_type::f32);
  unsigned char *color_px = out.color.data();
  float *depth_px = reinterpret_cast<float *>(out.depth.data());

  const render_settings &rs = _settings;
  const std::size_t nvol = prep.size();

  // Per-tile scratch, reused across the tile's rays: the spline-gradient
  // neighborhood cache (valid across rays -- it is keyed on the cell index)
  // and the per-ray last-cell tracker.
  struct tile_scratch {
    std::vector<detail::spline_gradient_cache> spline;
    std::vector<std::array<std::int64_t, 3>> last_cell;
  };

  const auto render_ray = [&](int px, int py, tile_scratch &scratch) {
    unsigned char *cpx = color_px + (std::size_t(py) * width + px) * 4;
    float *dpx = depth_px + (std::size_t(py) * width + px);
    *dpx = std::numeric_limits<float>::infinity();

    const ray r = rays.at(px, py);
    double t0 = 0.0, t1 = 0.0;
    if (!intersect_box(r, scene, t0, t1)) {
      cpx[0] = to_byte(rs.background[0]);
      cpx[1] = to_byte(rs.background[1]);
      cpx[2] = to_byte(rs.background[2]);
      cpx[3] = 0;
      return;
    }

    const vec3d view_vec = -r.direction; // toward the viewer, unit
    const double z_scale = dot(r.direction, rays.forward);

    float acc_r = 0.f, acc_g = 0.f, acc_b = 0.f, acc_a = 0.f;
    bool depth_set = false;

    for (std::size_t v = 0; v < nvol; ++v)
      scratch.last_cell[v] = {-1, -1, -1};

    // Front-to-back associated-color compositing; also latches the depth map
    // the first time accumulated alpha crosses the threshold.
    const auto composite = [&](const std::array<float, 3> &c, float a, double t) {
      const float ratio = a * (1.f - acc_a);
      acc_r += c[0] * ratio;
      acc_g += c[1] * ratio;
      acc_b += c[2] * ratio;
      acc_a += ratio;
      if (!depth_set && acc_a >= rs.depth_alpha_threshold) {
        *dpx = float(t * z_scale);
        depth_set = true;
      }
    };

    for (double t = t0; t <= t1 && acc_a < rs.opacity_cutoff; t += unit_step) {
      const vec3d pnt = r.origin + r.direction * t;
      if (!rs.cut_planes.empty() && culled_by_planes(rs.cut_planes, pnt))
        continue;

      for (std::size_t v = 0; v < nvol; ++v) {
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

        // 1. Isosurfaces (legacy ISO_SURFACE): MC intersection in this cell.
        if (p.any_iso) {
          fetch_corners();
          detail::mc_cell cell;
          cell.id[0] = idx[0];
          cell.id[1] = idx[1];
          cell.id[2] = idx[2];
          cell.orig = p.grid.minb; // THIS volume's frame (legacy used env[0]'s)
          cell.span = p.grid.span;
          cell.from_binary_corners(vals);
          // Intersect in local space.  The local direction is NOT normalized,
          // so the affine map preserves the ray parameter: t_hit is directly
          // comparable to the world-space march t (and the depth map).
          const ray local_ray{p.to_local_point(r.origin), p.to_local_vector(r.direction)};
          for (const isosurface &surf : vs.isosurfaces) {
            if (surf.value < min_val || surf.value > max_val)
              continue;
            float w[3];
            double t_hit = 0.0;
            if (!detail::intersect_isosurface_in_cell(local_ray, float(surf.value), cell, w, t_hit))
              continue;
            // An isosurface hit latches the depth map on its own -- the
            // frame contract is "first iso hit or first threshold-crossing
            // sample, whichever comes first".
            if (!depth_set) {
              *dpx = float(t_hit * z_scale);
              depth_set = true;
            }
            const vec3d grad = scratch.spline[v].evaluate(p.grid, idx, w);
            const vec3d normal = normalized(p.normal_to_world(grad));
            const std::array<float, 3> shaded =
                detail::blinn_phong(surf.color, normal, view_vec, rs.lights, rs.two_sided_lighting,
                                    rs.ambient, surf.shininess);
            composite(shaded, surf.opacity, t_hit);
          }
        }

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
            const std::array<float, 3> shaded =
                detail::blinn_phong({s.r, s.g, s.b}, normal, view_vec, rs.lights,
                                    rs.two_sided_lighting, rs.ambient, defaults::shininess);
            composite(shaded, a, t);
          }
        }
      }
    }

    // Over-blend the remaining transparency with the background (replaces the
    // legacy divide-by-alpha normalization of saturated rays).
    const float rest = 1.f - acc_a;
    cpx[0] = to_byte(acc_r + rs.background[0] * rest);
    cpx[1] = to_byte(acc_g + rs.background[1] * rest);
    cpx[2] = to_byte(acc_b + rs.background[2] * rest);
    cpx[3] = to_byte(acc_a);
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
