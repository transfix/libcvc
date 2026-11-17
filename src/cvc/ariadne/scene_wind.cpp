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

// scene_wind.cpp — the scene's wind, as a node reads it (see scene_wind.h).

#include <algorithm>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_wind.h>

namespace cvc {
namespace ariadne {

cvc::algo::wind_state SceneWind::state() const {
  return {std::max(speed, 0.0), dir * 3.14159265358979323846 / 180.0, std::max(gustiness, 0.0)};
}

SceneWind scene_wind(cvc::app &app, const SceneNode &n, const std::string &bind_prefix,
                     const SceneWindKeys &keys, const SceneWind &defaults) {
  auto key = [&](const char *k, const char *prop, double def) {
    const std::string path = resolve_bind(bind_prefix, node_key(n, k));
    const double seeded = n.props.num(prop, def);
    read_or_seed<double>(app, path, seeded);
    return read_or<double>(app, path, seeded);
  };
  SceneWind w;
  w.speed = std::max(key(keys.speed, keys.speed_prop, defaults.speed), 0.0);
  w.dir = key(keys.dir, keys.dir_prop, defaults.dir);
  w.gustiness = std::max(key(keys.gustiness, keys.gustiness_prop, defaults.gustiness), 0.0);
  return w;
}

} // namespace ariadne
} // namespace cvc
