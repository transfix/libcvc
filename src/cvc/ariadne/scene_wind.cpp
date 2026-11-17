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
  if (const std::optional<SceneWind> scene = scene_wind_set(app, bind_prefix))
    return *scene;
  return own;
}

std::shared_ptr<const cvc::physics::wind_flow> scene_wind_flow(const SceneWind &w, double base_z) {
  return std::make_shared<const cvc::physics::wind_flow>(w.state(), base_z);
}

} // namespace ariadne
} // namespace cvc
