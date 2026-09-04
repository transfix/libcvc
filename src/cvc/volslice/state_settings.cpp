#include <algorithm>
#include <cctype>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/volslice/state_settings.h>
#include <sstream>
#include <stdexcept>

namespace cvc {
namespace volslice {

namespace {

std::string csv(const std::vector<double> &vals) {
  std::ostringstream out;
  out.precision(17); // round-trip doubles exactly
  for (std::size_t i = 0; i < vals.size(); ++i) {
    if (i)
      out << ",";
    out << vals[i];
  }
  return out.str();
}

// Strict CSV-of-doubles parse; throws std::invalid_argument on junk so
// handleStateChanged's try/catch leaves the object alone on partial state
// (the cvc::volren::state_settings contract).
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

// The shared VolumeNode/volren transfer-function encoding: merge a color ramp
// (value,r,g,b per point) and an opacity ramp (value,a per point) into one
// transfer_function with control points at the union of both scalars.
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

} // namespace

std::string state_settings::sceneStatePath(const std::string &prefix) {
  return prefix + cvc::state::SEPARATOR + "volslice";
}

state_settings::state_settings(cvc::app &ctx, const std::string &statePath,
                               std::function<void(const render_settings &)> apply)
    : cvc::state_object<state_settings>(ctx, statePath), _apply(std::move(apply)) {
  // Synchronous handlers: the apply callback pokes a renderer that is not
  // thread-safe, and destruction must not race a queued handler.
  this->setInstanceThreading(false);
  cvc::state_init_scope<state_settings> init(*this);
  seedState(_settings);
}

void state_settings::set(const render_settings &s) {
  {
    std::lock_guard<std::mutex> lock(_mutex);
    _settings = s;
  }
  // Object -> state only: suppress our own handler so apply() is not invoked
  // with the values the caller just handed us.
  cvc::state_init_scope<state_settings> init(*this);
  seedState(s);
}

render_settings state_settings::get() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _settings;
}

void state_settings::seedState(const render_settings &s) {
  getState("quality").value(s.slices.quality);
  getState("max_planes").value(s.slices.max_planes);
  getState("near_plane").value(s.slices.near_plane);
  getState("interpolation").value(int(s.filter));
  getState("opacity_correction").value(s.opacity_correction ? 1 : 0);
  getState("tf_auto_domain").value(s.tf_auto_domain ? 1 : 0);
  getState("window").value(s.window_min == 0.0 && s.window_max == 0.0
                               ? std::string()
                               : csv({s.window_min, s.window_max}));
  std::vector<double> color, opacity;
  split_ramps(s.tf, color, opacity);
  getState("transfer_function.color").value(csv(color));
  getState("transfer_function.opacity").value(csv(opacity));
}

bool state_settings::readAllFromState(render_settings &out) const {
  try {
    render_settings s;
    // Contract values read raw: compute_slices() clamps quality/near_plane
    // itself, so out-of-range values round-trip rather than being silently
    // rewritten (the volren `steps` convention).
    s.slices.quality = getState("quality").value<double>();
    s.slices.near_plane = getState("near_plane").value<double>();
    // max_planes is an implementation resource (it caps a per-frame vertex
    // allocation), so it is CLAMPED on read like volren's shadows.resolution.
    s.slices.max_planes =
        std::max(limits::min_max_planes,
                 std::min(getState("max_planes").value<int>(), limits::max_max_planes));

    // Enums REJECT unknown values (all-or-nothing: keep the last good state).
    const int filter = getState("interpolation").value<int>();
    if (filter != int(interpolation::linear) && filter != int(interpolation::nearest))
      throw std::invalid_argument("interpolation must be 0 (linear) or 1 (nearest)");
    s.filter = interpolation(filter);

    s.opacity_correction = getState("opacity_correction").value<int>() != 0;
    s.tf_auto_domain = getState("tf_auto_domain").value<int>() != 0;

    const std::string window = getState("window").value();
    if (!window.empty()) {
      const std::vector<double> w = parse_csv(window);
      if (w.size() != 2)
        throw std::invalid_argument("window must be \"\" or \"min,max\"");
      s.window_min = w[0];
      s.window_max = w[1];
    }

    s.tf = merge_ramps(parse_csv(getState("transfer_function.color").value()),
                       parse_csv(getState("transfer_function.opacity").value()));

    out = s;
    return true;
  } catch (...) {
    // Partial or invalid state (a peer mid-write, a typo in a script): keep
    // the last good settings.
    return false;
  }
}

void state_settings::handleStateChanged(const std::string &) {
  render_settings next;
  if (!readAllFromState(next))
    return;
  {
    std::lock_guard<std::mutex> lock(_mutex);
    _settings = next;
  }
  if (_apply)
    _apply(next);
}

} // namespace volslice
} // namespace cvc
