/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

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

// scene_water.cpp — a water node's physics (see scene_water.h).

#include <algorithm>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_generated_world.h> // SceneColliders
#include <cvc/ariadne/scene_water.h>
#include <cvc/geometry/height_field.h>
#include <cvc/ocean/h0.h>
#include <cvc/ocean/sea_surface.h>
#include <cvc/ocean/spectrum.h>
#include <cvc/physics/field.h>
#include <cvc/physics/ocean_surface.h>
#include <cvc/volume/volume.h>
#include <cvc/world/world_geometry.h>

namespace cvc {
namespace ariadne {

namespace {

constexpr double kTiles[3] = {251.0, 83.0, 29.0};  // the drawn cascades' tiles (water_node.cpp)
constexpr std::uint64_t kCascadeSeed = 0xC0A57ull; // and their seed
constexpr int kH0N = 256;                          // and their texels
constexpr double kPi = 3.14159265358979323846;

} // namespace

SceneWaterSpec scene_water_spec(cvc::app &app, const SceneNode &n, const std::string &bind_prefix) {
  SceneWaterSpec s;
  s.level = n.props.num("level", 0.0);
  s.outer = std::max(n.props.num("outer", 6000.0), 0.0);
  auto key = [&](const char *k, const char *prop, double def) {
    const std::string path = resolve_bind(bind_prefix, node_key(n, k));
    const double seeded = n.props.num(prop, def);
    read_or_seed<double>(app, path, seeded);
    return read_or<double>(app, path, seeded);
  };
  s.wind_speed = std::max(key("water.wind_speed", "wind_speed", s.wind_speed), 0.0);
  s.wind_dir = key("water.wind_dir", "wind_dir", s.wind_dir);
  s.fetch = std::max(key("water.fetch", "fetch", s.fetch), 400.0);
  s.chop = key("water.chop", "chop", s.chop);
  s.wave_amp = key("water.wave_amp", "wave_amp", s.wave_amp);
  return s;
}

bool water_rim_is_sea(const cvc::height_field &hf, double level) {
  const int rows = hf.rows(), cols = hf.cols();
  std::size_t wet = 0, all = 0;
  auto count = [&](int r, int c) {
    ++all;
    if (hf.height_at(r, c) < level)
      ++wet;
  };
  for (int c = 0; c < cols; ++c) {
    count(0, c);
    count(rows - 1, c);
  }
  for (int r = 1; r < rows - 1; ++r) {
    count(r, 0);
    count(r, cols - 1);
  }
  return all > 0 && wet >= std::size_t(0.98 * double(all));
}

struct WaterRuntime::impl {
  cvc::app *app = nullptr;
  SceneNode node;
  std::string prefix;
  std::function<std::shared_ptr<const cvc::height_field>()> ground;
  std::shared_ptr<SceneColliders> colliders = std::make_shared<SceneColliders>();
  std::vector<double> built;   // the spec the field was built for
  std::vector<double> windSea; // the wind h0 was built for
  std::vector<cvc::ocean::h0_field> h0;
  const float *groundData = nullptr; // the ground the field was built over (a regenerate swaps it)
  std::size_t groundSize = 0;
};

WaterRuntime::WaterRuntime(cvc::app &app, const SceneNode &n, const std::string &bind_prefix,
                           std::function<std::shared_ptr<const cvc::height_field>()> ground)
    : _impl(std::make_unique<impl>()) {
  _impl->app = &app;
  _impl->node = n;
  _impl->prefix = bind_prefix;
  _impl->ground = std::move(ground);
  tick();
}
WaterRuntime::~WaterRuntime() = default;

const std::string &WaterRuntime::node() const { return _impl->node.id; }
const std::shared_ptr<SceneColliders> &WaterRuntime::colliders() const { return _impl->colliders; }

bool WaterRuntime::tick() {
  impl &d = *_impl;
  const std::shared_ptr<const cvc::height_field> g = d.ground ? d.ground() : nullptr;
  if (!g || g->rows() < 2 || g->cols() < 2)
    return false;
  const SceneWaterSpec s = scene_water_spec(*d.app, d.node, d.prefix);
  const std::vector<double> spec{s.level, s.wind_speed, s.wind_dir, s.fetch,
                                 s.chop,  s.wave_amp,   s.outer};
  const std::vector<float> &heights = g->heights();
  const bool groundNew = heights.data() != d.groundData || heights.size() != d.groundSize;
  if (!groundNew && spec == d.built && d.colliders->geometry)
    return false;

  // The cascades' h0 again only when the wind changed (tens of milliseconds).
  const std::vector<double> wind{s.wind_speed, s.wind_dir, s.fetch};
  if (wind != d.windSea || d.h0.empty()) {
    cvc::ocean::sea_state sea;
    sea.wind.speed_mps = s.wind_speed;
    sea.wind.dir_rad = s.wind_dir * kPi / 180.0;
    sea.wind.fetch_m = s.fetch;
    sea.swell.hs_m = 0.0; // the wind sea alone, as the drawn cascades
    d.h0.clear();
    for (const cvc::ocean::cascade &c :
         cvc::ocean::make_cascades({kTiles[0], kTiles[1], kTiles[2]}, kCascadeSeed))
      d.h0.push_back(cvc::ocean::build_h0(kH0N, c, sea));
    d.windSea = wind;
  }

  // The sea the renderer draws, on the CPU (the open sea's: no shore fade yet).
  std::vector<cvc::ocean::sea_cascade> cs;
  for (const cvc::ocean::h0_field &h : d.h0)
    cs.push_back({h, 0.0, 0.0});
  cvc::ocean::sea_params p;
  p.level = s.level;
  p.depth_m = 1000.0;
  p.wave_amp = s.wave_amp;
  p.chop = s.chop;
  p.source_cell_m = 0.5; // B3's source on a 0.5 m lattice, as lsystem_coast's debris
  auto sea = std::make_shared<const cvc::ocean::sea_surface>(std::move(cs), p);

  // One field over the water: the ground, and `outer` past it when its rim is sea; from below the
  // deepest ground to above the highest crest.
  const double m = water_rim_is_sea(*g, s.level) ? s.outer : 0.0;
  const float lowest = *std::min_element(heights.begin(), heights.end());
  auto box = std::make_shared<cvc::volume>(
      *d.app, cvc::dimension(2, 2, 2), cvc::Float,
      cvc::bounding_box(g->x0() - m, g->y0() - m, std::min<double>(lowest, s.level) - 1.0,
                        g->max_x() + m, g->max_y() + m, s.level + 15.0));
  box->fill(1000.0); // kg/m^3
  cvc::world::volume_item item;
  item.name = d.node.id + "_water";
  item.field.kind = cvc::physics::field_kind::fluid;
  item.field.volume = box;
  item.field.drag_coefficient = 1.0;
  item.field.damping = 1.0e4; // a metre-sized body follows the water within a tenth of a second
  item.field.surface = std::make_shared<cvc::physics::ocean_surface>(sea);
  auto geo = std::make_shared<cvc::world::world_geometry>();
  geo->add_volume(std::move(item));
  d.colliders->geometry = geo;
  ++d.colliders->revision;
  d.built = spec;
  d.groundData = heights.data();
  d.groundSize = heights.size();
  return true;
}

} // namespace ariadne
} // namespace cvc
