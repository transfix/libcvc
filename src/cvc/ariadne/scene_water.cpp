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

// scene_water.cpp — a water node's physics and coast (see scene_water.h).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_generated_world.h> // SceneColliders
#include <cvc/ariadne/scene_water.h>
#include <cvc/geometry/height_field.h>
#include <cvc/ocean/h0.h>
#include <cvc/ocean/sea_surface.h>
#include <cvc/ocean/shore.h>
#include <cvc/ocean/spectrum.h>
#include <cvc/physics/field.h>
#include <cvc/physics/ocean_surface.h>
#include <cvc/volume/volume.h>
#include <cvc/world/grid.h>
#include <cvc/world/water_bodies.h>
#include <cvc/world/world_geometry.h>

namespace cvc {
namespace ariadne {

namespace {

constexpr double kTiles[3] = {251.0, 83.0, 29.0};  // the drawn cascades' tiles (water_node.cpp)
constexpr std::uint64_t kCascadeSeed = 0xC0A57ull; // and their seed
constexpr int kH0N = 256;                          // and their texels
constexpr double kPi = 3.14159265358979323846;
constexpr int kShoreMaxSide = 1025;             // the shore grid's most samples a side
constexpr double kShoreMarginMax = 512.0;       // m: the most it goes past a rim of sea
constexpr std::uint64_t kSwellSeed = 0x5E11ull; // the trains' phases (lsystem_coast's)

// The wind sea and the swell of spec s. The cascades draw the wind sea alone (their h0 is built
// with the swell's height 0); the shore layer draws the swell.
cvc::ocean::sea_state sea_state_of(const SceneWaterSpec &s) {
  cvc::ocean::sea_state sea;
  sea.wind.speed_mps = s.wind_speed;
  sea.wind.dir_rad = s.wind_dir * kPi / 180.0;
  sea.wind.fetch_m = s.fetch;
  sea.swell.hs_m = std::max(s.swell_height, 0.0);
  sea.swell.tp_s = std::max(s.swell_period, 3.0);
  sea.swell.dir_rad = s.swell_dir * kPi / 180.0;
  return sea;
}

} // namespace

SceneWaterSpec scene_water_spec(cvc::app &app, const SceneNode &n, const std::string &bind_prefix) {
  SceneWaterSpec s;
  s.level = n.props.num("level", 0.0);
  s.outer = std::max(n.props.num("outer", 6000.0), 0.0);
  s.swash = n.props.num("swash", 1.0) != 0.0;
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
  s.shore_fade = std::max(key("water.shore_fade", "shore_fade", s.shore_fade), 0.05);
  s.swell_height = std::max(key("water.swell_height", "swell_height", s.swell_height), 0.0);
  s.swell_period = std::max(key("water.swell_period", "swell_period", s.swell_period), 3.0);
  s.swell_dir = key("water.swell_dir", "swell_dir", s.swell_dir);
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

std::vector<double> scene_water_shore_signature(const SceneWaterSpec &s) {
  const bool swell = s.swell_height > 0.0;
  return {s.level, swell ? s.swell_height : 0.0, swell ? s.swell_period : 0.0,
          swell ? s.swell_dir : 0.0};
}

std::shared_ptr<const SceneWaterShore> scene_water_shore(const cvc::height_field &hf,
                                                         const SceneWaterSpec &s) {
  const int rows = hf.rows(), cols = hf.cols();
  if (rows < 2 || cols < 2 || !(hf.dx() > 0.0) ||
      std::abs(hf.dx() - hf.dy()) > 1e-9 * std::max(hf.dx(), hf.dy()))
    return nullptr;
  const auto t0 = std::chrono::steady_clock::now();
  // Every `stride` samples, so neither side passes kShoreMaxSide.
  const int stride = std::max(
      1, static_cast<int>(std::ceil(double(std::max(rows, cols) - 1) / double(kShoreMaxSide - 1))));
  const int r1 = (rows - 1) / stride + 1, c1 = (cols - 1) / stride + 1;
  const double cell = hf.dx() * stride;
  // Past a rim of sea, one deep-water swell wavelength of the rim's ground (as the drawn bed
  // clamps), so the swell enters the grid before the coast.
  int m = 0;
  if (s.swell_height > 0.0 && water_rim_is_sea(hf, s.level)) {
    const double tp = std::max(s.swell_period, 3.0);
    const double lambda0 = cvc::ocean::gravity * tp * tp / (2.0 * kPi);
    m = static_cast<int>(std::ceil(std::min(lambda0, kShoreMarginMax) / cell));
  }
  const int R = r1 + 2 * m, C = c1 + 2 * m;
  std::vector<float> ground(std::size_t(R) * std::size_t(C));
  for (int r = 0; r < R; ++r) {
    const int sr = std::clamp(r - m, 0, r1 - 1) * stride;
    for (int c = 0; c < C; ++c)
      ground[std::size_t(r) * C + c] = hf.height_at(sr, std::clamp(c - m, 0, c1 - 1) * stride);
  }
  cvc::world::grid_spec g;
  g.rows = R;
  g.cols = C;
  g.min_x = hf.x0() - m * cell;
  g.min_y = hf.y0() - m * cell;
  g.max_x = g.min_x + (C - 1) * cell;
  g.max_y = g.min_y + (R - 1) * cell;
  cvc::world::water_bodies_params p;
  p.level_m = s.level;
  p.normal_smooth_m = std::max(8.0, 2.0 * cell); // lsystem_coast's, or two cells
  const cvc::world::water_bodies wb = cvc::world::build_water_bodies(std::move(ground), g, p);

  auto grid = std::make_shared<cvc::ocean::shore_grid>();
  grid->rows = R;
  grid->cols = C;
  grid->min_x = g.min_x;
  grid->min_y = g.min_y;
  grid->cell = cell;
  grid->depth = wb.depth;
  grid->shore_m = wb.shore_m;
  grid->normal_x = wb.normal_x;
  grid->normal_y = wb.normal_y;
  auto out = std::make_shared<SceneWaterShore>();
  out->slope = wb.slope;
  if (s.swell_height > 0.0) {
    const cvc::ocean::sea_state sea = sea_state_of(s);
    cvc::ocean::shore_swell sw;
    sw.hs_m = sea.swell.hs_m;
    sw.tp_s = sea.swell.tp_s;
    sw.dir_rad = sea.swell.dir_rad;
    cvc::ocean::shore_layer_params lp;
    lp.trains = 3; // the drawn water sums three
    lp.seed = kSwellSeed;
    out->layer = std::make_shared<const cvc::ocean::shore_layer>(
        cvc::ocean::shore_layer::build(*grid, sw, lp));
  }
  out->grid = std::move(grid);
  out->build_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  return out;
}

SceneWaterCascades scene_water_cascades(const SceneWaterSpec &s) {
  SceneWaterCascades out;
  out.seed = kCascadeSeed;
  const cvc::ocean::sea_state sea = sea_state_of(s);
  // The band's energy peaks at the sea's peak (the swell's, when there is one) if that falls inside
  // the band, else at the band's edge nearer it (lsystem_coast's cascadeFade, cascadeDecayK): a
  // wave feels the bottom in water shallower than half its length.
  double w = cvc::ocean::wind_sea_jonswap(sea.wind).peak_omega;
  if (sea.swell.hs_m > 1e-3)
    w = std::min(w, cvc::ocean::swell_jonswap(sea.swell).peak_omega);
  const double kPeak = w * w / cvc::ocean::gravity; // deep water
  for (const cvc::ocean::cascade &c :
       cvc::ocean::make_cascades({kTiles[0], kTiles[1], kTiles[2]}, kCascadeSeed)) {
    const double k = std::min(std::max(kPeak, c.k_min), c.k_max);
    out.tiles.push_back(c.tile_m);
    out.fade_m.push_back(std::max(0.05, std::min(s.shore_fade, kPi / k)));
    out.decay_k.push_back(k);
  }
  return out;
}

struct WaterRuntime::impl {
  cvc::app *app = nullptr;
  SceneNode node;
  std::string prefix;
  std::function<std::shared_ptr<const cvc::height_field>()> ground;
  std::shared_ptr<SceneColliders> colliders = std::make_shared<SceneColliders>();
  std::vector<double> built;    // the spec the field was built for
  std::vector<double> windSea;  // the wind h0 was built for
  std::vector<double> shoreSig; // and the coast
  std::vector<cvc::ocean::h0_field> h0;
  std::shared_ptr<const SceneWaterShore> shore;
  std::shared_ptr<const cvc::ocean::sea_surface> sea;
  std::shared_ptr<const cvc::physics::field> field;
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
std::shared_ptr<const SceneWaterShore> WaterRuntime::shore() const { return _impl->shore; }
std::shared_ptr<const cvc::ocean::sea_surface> WaterRuntime::sea() const { return _impl->sea; }
std::shared_ptr<const cvc::physics::field> WaterRuntime::field() const { return _impl->field; }
double WaterRuntime::level() const {
  const impl &d = *_impl;
  return d.built.empty() ? scene_water_spec(*d.app, d.node, d.prefix).level : d.built[0];
}

bool WaterRuntime::tick() {
  impl &d = *_impl;
  const std::shared_ptr<const cvc::height_field> g = d.ground ? d.ground() : nullptr;
  if (!g || g->rows() < 2 || g->cols() < 2)
    return false;
  const SceneWaterSpec s = scene_water_spec(*d.app, d.node, d.prefix);
  const std::vector<double> spec{s.level,        s.wind_speed, s.wind_dir,   s.fetch,
                                 s.chop,         s.wave_amp,   s.shore_fade, s.swell_height,
                                 s.swell_period, s.swell_dir,  s.outer,      s.swash ? 1.0 : 0.0};
  const std::vector<float> &heights = g->heights();
  const bool groundNew = heights.data() != d.groundData || heights.size() != d.groundSize;
  if (!groundNew && spec == d.built && d.colliders->geometry)
    return false;

  // The cascades' h0 again only when the wind changed (tens of milliseconds).
  const cvc::ocean::sea_state sea = sea_state_of(s);
  const std::vector<double> wind{s.wind_speed, s.wind_dir, s.fetch};
  if (wind != d.windSea || d.h0.empty()) {
    cvc::ocean::sea_state windOnly = sea;
    windOnly.swell.hs_m = 0.0; // the wind sea alone, as the drawn cascades
    d.h0.clear();
    for (const cvc::ocean::cascade &c :
         cvc::ocean::make_cascades({kTiles[0], kTiles[1], kTiles[2]}, kCascadeSeed))
      d.h0.push_back(cvc::ocean::build_h0(kH0N, c, windOnly));
    d.windSea = wind;
  }
  // The coast again when the ground, the level or the swell changed.
  const std::vector<double> shoreSig = scene_water_shore_signature(s);
  if (groundNew || shoreSig != d.shoreSig || !d.shore) {
    d.shore = scene_water_shore(*g, s);
    d.shoreSig = shoreSig;
  }

  // The sea the renderer draws, on the CPU: the cascades fading at the shore, the swell's shore
  // layer, and its swash on the beach, which the drawn node draws on the ground (`swash: 0`: none).
  const SceneWaterCascades cd = scene_water_cascades(s);
  std::vector<cvc::ocean::sea_cascade> cs;
  for (std::size_t i = 0; i < d.h0.size(); ++i)
    cs.push_back({d.h0[i], cd.fade_m[i], cd.decay_k[i]});
  cvc::ocean::sea_params p;
  p.level = s.level;
  p.depth_m = 1000.0;
  p.wave_amp = s.wave_amp;
  p.chop = s.chop;
  p.source_cell_m = 0.5; // B3's source on a 0.5 m lattice, as lsystem_coast's debris
  p.swash = s.swash;
  d.sea = std::make_shared<const cvc::ocean::sea_surface>(
      std::move(cs), p, d.shore ? d.shore->grid : nullptr, d.shore ? d.shore->layer : nullptr);

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
  item.field.surface = std::make_shared<cvc::physics::ocean_surface>(d.sea);
  d.field = std::make_shared<const cvc::physics::field>(item.field);
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
