#include <algorithm>
#include <cctype>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/volren/raycaster.h>
#include <cvc/volren/state_settings.h>
#include <sstream>
#include <stdexcept>

namespace cvc {
namespace volren {

namespace {

std::string csv(const std::vector<double> &vals) {
  std::ostringstream out;
  out.precision(17); // round-trip doubles exactly (default 6 truncated poses/matrices)
  for (std::size_t i = 0; i < vals.size(); ++i) {
    if (i)
      out << ",";
    out << vals[i];
  }
  return out.str();
}

// Strict CSV-of-doubles parse; throws std::invalid_argument on junk so
// handleStateChanged's try/catch leaves the object alone on partial state.
std::vector<double> parse_csv(const std::string &s) {
  std::vector<double> out;
  std::istringstream in(s);
  std::string tok;
  while (std::getline(in, tok, ',')) {
    std::size_t pos = 0;
    const double v = std::stod(tok, &pos);
    while (pos < tok.size() && std::isspace(static_cast<unsigned char>(tok[pos])))
      ++pos;
    if (pos != tok.size())
      throw std::invalid_argument("trailing junk in CSV number: " + tok);
    out.push_back(v);
  }
  return out;
}

std::string triple(const std::array<double, 3> &a) { return csv({a[0], a[1], a[2]}); }
std::string triple(const std::array<float, 3> &a) {
  return csv({double(a[0]), double(a[1]), double(a[2])});
}

std::array<double, 3> parse_triple(const std::string &s) {
  const std::vector<double> v = parse_csv(s);
  if (v.size() != 3)
    throw std::invalid_argument("expected 3 CSV values: " + s);
  return {v[0], v[1], v[2]};
}

std::array<float, 3> parse_triple_f(const std::string &s) {
  const std::array<double, 3> d = parse_triple(s);
  return {float(d[0]), float(d[1]), float(d[2])};
}

// Merge a VolumeNode-style color ramp (value,r,g,b per point) and opacity
// ramp (value,a per point) into the combined transfer_function: control
// points at the union of both ramps' scalars, each sampled from its ramp.
transfer_function merge_ramps(const std::vector<double> &color,
                              const std::vector<double> &opacity) {
  if (color.size() % 4 != 0)
    throw std::invalid_argument("transfer_function.color needs 4 values per point");
  if (opacity.size() % 2 != 0)
    throw std::invalid_argument("transfer_function.opacity needs 2 values per point");

  transfer_function color_tf, alpha_tf;
  for (std::size_t i = 0; i < color.size(); i += 4)
    color_tf.add({color[i], float(color[i + 1]), float(color[i + 2]), float(color[i + 3]), 0.f});
  for (std::size_t i = 0; i < opacity.size(); i += 2)
    alpha_tf.add({opacity[i], 0.f, 0.f, 0.f, float(opacity[i + 1])});

  std::vector<double> values;
  for (std::size_t i = 0; i < color.size(); i += 4)
    values.push_back(color[i]);
  for (std::size_t i = 0; i < opacity.size(); i += 2)
    values.push_back(opacity[i]);
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end()), values.end());

  transfer_function merged;
  for (const double v : values) {
    const rgba_f c = color_tf.sample(v);
    const rgba_f a = alpha_tf.sample(v);
    merged.add({v, c.r, c.g, c.b, a.a});
  }
  return merged;
}

void split_ramps(const transfer_function &tf, std::vector<double> &color,
                 std::vector<double> &opacity) {
  color.clear();
  opacity.clear();
  for (const transfer_point &p : tf.points()) {
    color.push_back(p.value);
    color.push_back(p.r);
    color.push_back(p.g);
    color.push_back(p.b);
    opacity.push_back(p.value);
    opacity.push_back(p.a);
  }
}

std::string volume_key(std::size_t i, const std::string &child) {
  return "volumes." + std::to_string(i) + "." + child;
}

} // namespace

std::string state_settings::sceneStatePath(const std::string &prefix) {
  return prefix + cvc::state::SEPARATOR + "volren";
}

state_settings::state_settings(cvc::app &ctx, const std::string &statePath,
                               std::function<void(const snapshot &)> apply)
    : cvc::state_object<state_settings>(ctx, statePath), _apply(std::move(apply)) {
  // Synchronous handlers: apply callbacks typically poke a renderer that is
  // not thread-safe, and destruction must not race a queued handler.
  this->setInstanceThreading(false);
  // state_init_scope suppresses this object's own handler (under the
  // state_object batch mutex, so it is safe against concurrent writers)
  // while the defaults are seeded -- the same mechanism CameraController
  // and FpsHud use.
  cvc::state_init_scope<state_settings> init(*this);
  seedState(_snap);
}

void state_settings::set(const snapshot &s) {
  {
    std::lock_guard<std::mutex> lock(_snapMutex);
    _snap = s;
  }
  // Object -> state only: suppress our own handler so apply() is not invoked
  // with the values the caller just handed us (the ShadowSettings::set rule).
  cvc::state_init_scope<state_settings> init(*this);
  seedState(s);
}

state_settings::snapshot state_settings::get() const {
  std::lock_guard<std::mutex> lock(_snapMutex);
  return _snap;
}

void state_settings::seedState(const snapshot &s) {
  const volren::camera &cam = s.camera;
  getState("camera.eye").value(triple(cam.eye));
  getState("camera.focal").value(triple(cam.focal));
  getState("camera.up").value(triple(cam.up));
  getState("camera.projection")
      .value(cam.projection == camera::projection_type::orthographic ? 1 : 0);
  getState("camera.vfov_degrees").value(cam.vfov_degrees);
  getState("camera.parallel_scale").value(cam.parallel_scale);
  getState("image.width").value(cam.width);
  getState("image.height").value(cam.height);

  const render_settings &rs = s.settings;
  getState("background").value(triple(rs.background));
  getState("steps").value(rs.steps);
  getState("opacity_cutoff").value(double(rs.opacity_cutoff));
  getState("depth_alpha_threshold").value(double(rs.depth_alpha_threshold));
  getState("two_sided_lighting").value(rs.two_sided_lighting ? 1 : 0);
  getState("ambient").value(double(rs.ambient));
  getState("ambient_hemisphere.enabled").value(rs.ambient_hemisphere.enabled ? 1 : 0);
  getState("ambient_hemisphere.sky").value(triple(rs.ambient_hemisphere.sky));
  getState("ambient_hemisphere.ground").value(triple(rs.ambient_hemisphere.ground));
  getState("ambient_hemisphere.up").value(triple(rs.ambient_hemisphere.up));
  getState("ao.strength").value(double(rs.ao.strength));
  getState("ao.radius").value(rs.ao.radius);
  getState("ao.samples").value(rs.ao.samples);
  getState("shading_gain").value(double(rs.shading_gain));
  getState("specular").value(double(rs.specular));
  getState("threads").value(int(rs.threads));
  getState("supersample").value(rs.supersample);

  const shadow_settings &sh = rs.shadows;
  getState("shadows.enabled").value(sh.enabled ? 1 : 0);
  getState("shadows.resolution").value(sh.resolution);
  getState("shadows.strength").value(double(sh.strength));
  getState("shadows.bias_scale").value(double(sh.bias_scale));
  getState("shadows.slope_scale").value(double(sh.slope_scale));
  getState("shadows.min_occluder_opacity").value(double(sh.min_occluder_opacity));
  getState("shadows.mode").value(sh.mode == shadow_mode::deep ? 1 : 0);
  getState("shadows.depth_slices").value(sh.depth_slices);
  getState("shadows.pcf_radius").value(double(sh.pcf_radius));
  getState("shadows.pcf_taps").value(sh.pcf_taps);
  {
    std::vector<double> indices;
    for (const int i : sh.lights)
      indices.push_back(double(i));
    getState("shadows.lights").value(csv(indices));
  }

  std::vector<double> flat;
  for (const light &l : rs.lights) {
    flat.push_back(l.color[0]);
    flat.push_back(l.color[1]);
    flat.push_back(l.color[2]);
    flat.push_back(l.direction[0]);
    flat.push_back(l.direction[1]);
    flat.push_back(l.direction[2]);
  }
  getState("lights").value(csv(flat));

  flat.clear();
  for (const cut_plane &c : rs.cut_planes) {
    flat.push_back(c.point[0]);
    flat.push_back(c.point[1]);
    flat.push_back(c.point[2]);
    flat.push_back(c.normal[0]);
    flat.push_back(c.normal[1]);
    flat.push_back(c.normal[2]);
  }
  getState("cut_planes").value(csv(flat));

  getState("volumes.count").value(int(s.volumes.size()));
  for (std::size_t i = 0; i < s.volumes.size(); ++i) {
    const volume_settings &vs = s.volumes[i];
    getState(volume_key(i, "shaded")).value(vs.shaded ? 1 : 0);
    getState(volume_key(i, "unshaded")).value(vs.unshaded ? 1 : 0);
    getState(volume_key(i, "tf_auto_domain")).value(vs.tf_auto_domain ? 1 : 0);
    getState(volume_key(i, "distance_field")).value(vs.distance_field ? 1 : 0);
    getState(volume_key(i, "matrix"))
        .value(csv(std::vector<double>(vs.model_transform.m.begin(), vs.model_transform.m.end())));

    std::vector<double> color, opacity;
    split_ramps(vs.tf, color, opacity);
    getState(volume_key(i, "transfer_function.color")).value(csv(color));
    getState(volume_key(i, "transfer_function.opacity")).value(csv(opacity));

    getState(volume_key(i, "window"))
        .value(vs.window_enabled ? csv({vs.window_min, vs.window_max}) : std::string());
    getState(volume_key(i, "gradient_ramp"))
        .value(vs.gradient_ramp.enabled ? csv({vs.gradient_ramp.ramp0, vs.gradient_ramp.ramp1,
                                               vs.gradient_ramp.ramp2, vs.gradient_ramp.plateau})
                                        : std::string());

    flat.clear();
    for (const isosurface &s : vs.isosurfaces) {
      flat.push_back(s.value);
      flat.push_back(s.opacity);
      flat.push_back(s.color[0]);
      flat.push_back(s.color[1]);
      flat.push_back(s.color[2]);
      flat.push_back(s.shininess);
    }
    getState(volume_key(i, "isosurfaces")).value(csv(flat));
  }
}

bool state_settings::readAllFromState(snapshot &out) const {
  try {
    volren::camera cam;
    cam.eye = parse_triple(getState("camera.eye").value());
    cam.focal = parse_triple(getState("camera.focal").value());
    cam.up = parse_triple(getState("camera.up").value());
    cam.projection = getState("camera.projection").value<int>() != 0
                         ? camera::projection_type::orthographic
                         : camera::projection_type::perspective;
    cam.vfov_degrees = getState("camera.vfov_degrees").value<double>();
    cam.parallel_scale = getState("camera.parallel_scale").value<double>();
    cam.width = getState("image.width").value<int>();
    cam.height = getState("image.height").value<int>();
    out.camera = cam;

    render_settings rs;
    rs.background = parse_triple_f(getState("background").value());
    rs.steps = getState("steps").value<int>();
    rs.opacity_cutoff = float(getState("opacity_cutoff").value<double>());
    rs.depth_alpha_threshold = float(getState("depth_alpha_threshold").value<double>());
    rs.two_sided_lighting = getState("two_sided_lighting").value<int>() != 0;
    rs.ambient = float(getState("ambient").value<double>());
    rs.ambient_hemisphere.enabled = getState("ambient_hemisphere.enabled").value<int>() != 0;
    rs.ambient_hemisphere.sky = parse_triple_f(getState("ambient_hemisphere.sky").value());
    rs.ambient_hemisphere.ground = parse_triple_f(getState("ambient_hemisphere.ground").value());
    rs.ambient_hemisphere.up = parse_triple(getState("ambient_hemisphere.up").value());
    // Read raw, like `supersample`: render() is the single place that decides
    // what is in range, so an out-of-range strength round-trips and is rejected
    // loudly there instead of being silently clamped into something that looks
    // like it worked.  `radius` has no range to violate (<= 0 is simply off).
    rs.ao.strength = float(getState("ao.strength").value<double>());
    rs.ao.radius = getState("ao.radius").value<double>();
    // Clamped on read like `shadows.resolution`: the tap count along the cone is
    // an implementation resource with a defensible range, not a contract.
    rs.ao.samples = std::max(limits::min_ao_samples,
                             std::min(getState("ao.samples").value<int>(), limits::max_ao_samples));
    // Neither is range-checked, for the same reason `ambient` is not: a gain
    // above 1 is an exposure choice and the per-channel clamp handles it.
    rs.shading_gain = float(getState("shading_gain").value<double>());
    rs.specular = float(getState("specular").value<double>());
    rs.threads = unsigned(std::max(0, getState("threads").value<int>()));
    // Read raw, exactly like `steps`: render() is the single place that decides
    // what is in range, so an out-of-range write round-trips and is rejected
    // loudly there instead of being silently clamped into something that looks
    // like it worked.
    rs.supersample = getState("supersample").value<int>();

    rs.shadows.enabled = getState("shadows.enabled").value<int>() != 0;
    // Clamped on read, the way `threads` is: a light-view raster is an
    // implementation resource with a defensible range, not a contract the
    // caller can violate meaningfully.
    rs.shadows.resolution =
        std::max(limits::min_shadow_resolution,
                 std::min(getState("shadows.resolution").value<int>(), limits::max_raster_dim));
    rs.shadows.strength = float(getState("shadows.strength").value<double>());
    rs.shadows.bias_scale = float(getState("shadows.bias_scale").value<double>());
    rs.shadows.slope_scale = float(getState("shadows.slope_scale").value<double>());
    rs.shadows.min_occluder_opacity =
        float(getState("shadows.min_occluder_opacity").value<double>());
    // An ENUM, so anything outside its domain is malformed state rather than a
    // value to clamp -- the shadows.lights discipline, not the resolution one.
    {
      const int m = getState("shadows.mode").value<int>();
      if (m != 0 && m != 1)
        return false;
      rs.shadows.mode = m == 1 ? shadow_mode::deep : shadow_mode::hard;
    }
    // Clamped on read like `resolution`, and for the same reason: the slice
    // count is an implementation resource with a defensible range (it bounds
    // the map's memory), not a contract the caller can violate meaningfully.
    rs.shadows.depth_slices = std::max(
        limits::min_shadow_depth_slices,
        std::min(getState("shadows.depth_slices").value<int>(), limits::max_shadow_depth_slices));
    // Both clamped on read like `resolution`: the filter's footprint and tap
    // count are implementation resources with defensible ranges.  A negative
    // radius clamps to 0, which is "unfiltered" -- the same thing render() does
    // with it, so the state and the renderer agree about what it means.
    rs.shadows.pcf_radius =
        float(std::max(0.0, std::min(getState("shadows.pcf_radius").value<double>(),
                                     double(limits::max_pcf_radius))));
    rs.shadows.pcf_taps =
        std::max(limits::min_pcf_taps,
                 std::min(getState("shadows.pcf_taps").value<int>(), limits::max_pcf_taps));
    for (const double v : parse_csv(getState("shadows.lights").value())) {
      // A light INDEX, so anything non-integral or negative is malformed
      // state, not a value to round -- leave the object alone.
      if (!(v == std::floor(v)) || v < 0.0 || v > double(limits::max_raster_dim))
        return false;
      rs.shadows.lights.push_back(int(v));
    }

    std::vector<double> flat = parse_csv(getState("lights").value());
    if (flat.size() % 6 != 0)
      return false;
    for (std::size_t i = 0; i < flat.size(); i += 6) {
      light l;
      l.color = {float(flat[i]), float(flat[i + 1]), float(flat[i + 2])};
      l.direction = {flat[i + 3], flat[i + 4], flat[i + 5]};
      rs.lights.push_back(l);
    }

    flat = parse_csv(getState("cut_planes").value());
    if (flat.size() % 6 != 0)
      return false;
    for (std::size_t i = 0; i < flat.size(); i += 6) {
      cut_plane c;
      c.point = {flat[i], flat[i + 1], flat[i + 2]};
      c.normal = {flat[i + 3], flat[i + 4], flat[i + 5]};
      rs.cut_planes.push_back(c);
    }
    out.settings = rs;

    const int count = std::max(0, getState("volumes.count").value<int>());
    out.volumes.clear();
    for (int i = 0; i < count; ++i) {
      volume_settings vs;
      vs.shaded = getState(volume_key(i, "shaded")).value<int>() != 0;
      vs.unshaded = getState(volume_key(i, "unshaded")).value<int>() != 0;
      vs.tf_auto_domain = getState(volume_key(i, "tf_auto_domain")).value<int>() != 0;
      // Read TOLERANTLY, unlike the per-volume keys above, because this one is
      // NEW: a per-volume key exists in the tree only once something seeded it,
      // so a tree written by an older build -- or by a peer that drives
      // `volumes.<i>.*` by hand -- simply has no node here, and value<int>() on
      // an empty node throws, which would reject the WHOLE snapshot rather than
      // one field.  Absent means the default; junk still throws and is rejected.
      const std::string df = getState(volume_key(i, "distance_field")).value();
      vs.distance_field = df.empty() ? false : parse_csv(df).at(0) != 0.0;

      const std::vector<double> matrix = parse_csv(getState(volume_key(i, "matrix")).value());
      if (matrix.size() == 16)
        vs.model_transform = mat4::from_row_major(matrix.data());
      else if (!matrix.empty())
        return false; // empty keeps identity

      vs.tf = merge_ramps(parse_csv(getState(volume_key(i, "transfer_function.color")).value()),
                          parse_csv(getState(volume_key(i, "transfer_function.opacity")).value()));

      const std::vector<double> window = parse_csv(getState(volume_key(i, "window")).value());
      if (window.size() == 2) {
        vs.window_enabled = true;
        vs.window_min = window[0];
        vs.window_max = window[1];
      } else if (!window.empty()) {
        return false;
      }

      const std::vector<double> ramp = parse_csv(getState(volume_key(i, "gradient_ramp")).value());
      if (ramp.size() == 3 || ramp.size() == 4) {
        vs.gradient_ramp.enabled = true;
        vs.gradient_ramp.ramp0 = ramp[0];
        vs.gradient_ramp.ramp1 = ramp[1];
        vs.gradient_ramp.ramp2 = ramp[2];
        if (ramp.size() == 4)
          vs.gradient_ramp.plateau = ramp[3];
      } else if (!ramp.empty()) {
        return false;
      }

      flat = parse_csv(getState(volume_key(i, "isosurfaces")).value());
      if (flat.size() % 6 != 0)
        return false;
      for (std::size_t j = 0; j < flat.size(); j += 6) {
        isosurface s;
        s.value = flat[j];
        s.opacity = float(flat[j + 1]);
        s.color = {float(flat[j + 2]), float(flat[j + 3]), float(flat[j + 4])};
        s.shininess = float(flat[j + 5]);
        vs.isosurfaces.push_back(s);
      }
      out.volumes.push_back(std::move(vs));
    }
    return true;
  } catch (const std::exception &) {
    return false; // partially-initialised or malformed state: leave the object alone
  }
}

void state_settings::handleStateChanged(const std::string &) {
  snapshot next;
  if (!readAllFromState(next))
    return;
  {
    std::lock_guard<std::mutex> lock(_snapMutex);
    _snap = next;
  }
  if (_apply)
    _apply(next);
}

void state_settings::apply_to(raycaster &rc) const {
  const snapshot s = get();
  rc.view() = s.camera;
  rc.settings() = s.settings;
  const std::size_t n = std::min(s.volumes.size(), rc.volume_count());
  for (std::size_t i = 0; i < n; ++i)
    rc.volume_config(i) = s.volumes[i];
}

} // namespace volren
} // namespace cvc
