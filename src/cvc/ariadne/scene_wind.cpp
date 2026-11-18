/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// scene_wind.cpp — the scene's one wind (see scene_wind.h).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_wind.h>
#include <cvc/physics/flow_velocity.h>
#include <cvc/state/state.h>

namespace cvc {
namespace ariadne {

namespace {

// A key's value as a number, when it is set and reads as one.
std::optional<double> number_at(cvc::app &app, const std::string &path) {
  cvc::state *s = cvc::state::instance(app).findDescendant(path);
  if (!s)
    return std::nullopt;
  const std::string v = s->value();
  if (v.empty())
    return std::nullopt;
  try {
    const double d = std::stod(v);
    if (std::isfinite(d))
      return d;
  } catch (const std::exception &) {
  }
  return std::nullopt;
}

} // namespace

cvc::algo::wind_state SceneWind::state() const {
  return {std::max(speed, 0.0), dir * 3.14159265358979323846 / 180.0, std::max(gustiness, 0.0)};
}

double SceneWind::mean_speed() const {
  if (front_x == 0.0 && front_y == 0.0)
    return speed;
  const double a = dir * 3.14159265358979323846 / 180.0, s = std::max(speed, 0.0);
  return std::hypot(s * std::cos(a) + front_x, s * std::sin(a) + front_y);
}

double SceneWind::mean_dir() const {
  if (front_x == 0.0 && front_y == 0.0)
    return dir;
  constexpr double kPi = 3.14159265358979323846;
  const double a = dir * kPi / 180.0, s = std::max(speed, 0.0);
  const double x = s * std::cos(a) + front_x, y = s * std::sin(a) + front_y;
  if (x == 0.0 && y == 0.0)
    return dir; // the front cancels the wind here: no way it blows, keep the authored one
  // The authored direction's turn, so a dir of 390 stays near 390 and a reader's comparisons hold.
  double turn = std::atan2(y, x) * 180.0 / kPi - dir;
  turn -= 360.0 * std::floor((turn + 180.0) / 360.0);
  return dir + turn;
}

void seed_scene_wind(cvc::app &app, const Scene &scene, const std::string &bind_prefix) {
  if (!scene.wind.present)
    return;
  read_or_seed<double>(app, resolve_bind(bind_prefix, "wind.speed"), scene.wind.speed);
  read_or_seed<double>(app, resolve_bind(bind_prefix, "wind.dir"), scene.wind.dir);
  read_or_seed<double>(app, resolve_bind(bind_prefix, "wind.gustiness"), scene.wind.gustiness);
}

std::optional<SceneWind> scene_wind_set(cvc::app &app, const std::string &bind_prefix) {
  const std::optional<double> speed = number_at(app, resolve_bind(bind_prefix, "wind.speed"));
  if (!speed)
    return std::nullopt;
  SceneWind w;
  w.speed = std::max(*speed, 0.0);
  w.dir = number_at(app, resolve_bind(bind_prefix, "wind.dir")).value_or(0.0);
  w.gustiness =
      std::max(number_at(app, resolve_bind(bind_prefix, "wind.gustiness")).value_or(0.25), 0.0);
  return w;
}

std::uint64_t scene_clock_period_ticks(double period, double dt) {
  return std::max<std::uint64_t>(
      1, static_cast<std::uint64_t>(std::llround(std::max(period, 0.0) / std::max(dt, 1e-9))));
}

std::optional<SceneWind> scene_wind_now(cvc::app &app, const std::string &bind_prefix) {
  std::optional<SceneWind> w = scene_wind_set(app, bind_prefix);
  if (!w)
    return w;
  cvc::state *s =
      cvc::state::instance(app).findDescendant(resolve_bind(bind_prefix, kSceneWindFront));
  if (!s)
    return w;
  const std::string v = s->value();
  double x = 0.0, y = 0.0;
  if (!v.empty() && std::sscanf(v.c_str(), "%lf %lf", &x, &y) == 2 && std::isfinite(x) &&
      std::isfinite(y)) {
    w->front_x = x;
    w->front_y = y;
  }
  return w;
}

bool write_scene_wind_front(cvc::app &app, const std::string &bind_prefix, double x, double y) {
  char buf[64] = "";
  // To a millimetre a second: a front that has not moved by that writes nothing.
  x = std::round(x * 1000.0) / 1000.0;
  y = std::round(y * 1000.0) / 1000.0;
  if (x != 0.0 || y != 0.0)
    std::snprintf(buf, sizeof buf, "%.3f %.3f", x + 0.0, y + 0.0);
  const std::string path = resolve_bind(bind_prefix, kSceneWindFront);
  cvc::state *s = cvc::state::instance(app).findDescendant(path);
  const std::string text(buf);
  if (s ? s->value() == text : text.empty())
    return false;
  cvc::state::instance(app)(path).value(text);
  return true;
}

SceneWind scene_wind(cvc::app &app, const SceneNode &n, const std::string &bind_prefix,
                     const SceneWindKeys &keys, const SceneWind &defaults) {
  auto key = [&](const char *k, const char *prop, double def) {
    const std::string path = resolve_bind(bind_prefix, node_key(n, k));
    const double seeded = n.props.num(prop, def);
    read_or_seed<double>(app, path, seeded);
    return read_or<double>(app, path, seeded);
  };
  // The node's own keys are seeded either way, so they are there to fall back to.
  SceneWind own;
  own.speed = std::max(key(keys.speed, keys.speed_prop, defaults.speed), 0.0);
  own.dir = key(keys.dir, keys.dir_prop, defaults.dir);
  own.gustiness = std::max(key(keys.gustiness, keys.gustiness_prop, defaults.gustiness), 0.0);
  if (const std::optional<SceneWind> scene = scene_wind_now(app, bind_prefix))
    return *scene;
  return own;
}

std::shared_ptr<const cvc::physics::wind_flow> scene_wind_flow(const SceneWind &w, double base_z) {
  return std::make_shared<const cvc::physics::wind_flow>(w.state(), base_z);
}

} // namespace ariadne
} // namespace cvc
