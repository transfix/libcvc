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
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_generated_world.h> // SceneColliders
#include <cvc/ariadne/scene_water.h>
#include <cvc/ariadne/scene_wind.h>
#include <cvc/ariadne/shown_clock.h> // the time a scene is drawn at (OW5q)
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/height_field.h>
#include <cvc/ocean/current.h>
#include <cvc/ocean/fetch.h>
#include <cvc/ocean/h0.h>
#include <cvc/ocean/sea_lag.h>
#include <cvc/ocean/sea_surface.h>
#include <cvc/ocean/shore.h>
#include <cvc/ocean/spectrum.h>
#include <cvc/physics/field.h>
#include <cvc/physics/ocean_surface.h>
#include <cvc/physics/shore_mask.h>
#include <cvc/volume/volume.h>
#include <cvc/world/grid.h>
#include <cvc/world/water_bodies.h>
#include <cvc/world/world_geometry.h>
#include <optional>

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
// The swell's least share on a lee shore (shore_layer_params::lee_exposure): what diffraction
// carries round an island, so its far side has small breakers and a gentle swash, not a dead calm.
constexpr double kSwellLee = 0.25;

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

constexpr int kFetchMaxSide = 256; // the fetch map's most samples a side (a coarse texture)

// The cascades' bands (lsystem_coast's tiles and seed).
std::vector<cvc::ocean::cascade> water_bands() {
  return cvc::ocean::make_cascades({kTiles[0], kTiles[1], kTiles[2]}, kCascadeSeed);
}

// The cascades' h0 for keyframe wind w: the wind sea alone, as the drawn cascades.
std::vector<cvc::ocean::h0_field> keyframe_h0(const cvc::ocean::sea_wind &w) {
  cvc::ocean::sea_state sea;
  sea.wind.speed_mps = w.speed_mps;
  sea.wind.dir_rad = w.dir_rad;
  sea.wind.fetch_m = w.fetch_m;
  sea.swell.hs_m = 0.0;
  std::vector<cvc::ocean::h0_field> out;
  for (const cvc::ocean::cascade &c : water_bands())
    out.push_back(cvc::ocean::build_h0(kH0N, c, sea));
  return out;
}

// Each cascade's fully grown fetch for keyframe wind w (fetch.h).
std::vector<double> keyframe_full(const cvc::ocean::sea_wind &w) {
  const double F = cvc::ocean::spectrum_fetch(w.speed_mps, w.fetch_m);
  std::vector<double> out;
  for (const cvc::ocean::cascade &c : water_bands())
    out.push_back(cvc::ocean::band_full_fetch(w.speed_mps, c.k_min, F));
  return out;
}

// The fetch map over the coast for a wind toward dir_rad: coarse (kFetchMaxSide a side at most),
// each sample capped by its body's open extent, the sea's by the open fetch.
std::shared_ptr<const cvc::ocean::fetch_field> keyframe_fetch(const SceneWaterShore &sh,
                                                              double dir_rad, double open_m) {
  if (!sh.grid)
    return nullptr;
  const cvc::ocean::shore_grid &g = *sh.grid;
  const int stride = std::max(1, static_cast<int>(std::ceil(double(std::max(g.rows, g.cols) - 1) /
                                                            double(kFetchMaxSide - 1))));
  return std::make_shared<const cvc::ocean::fetch_field>(cvc::ocean::build_fetch(
      g, dir_rad, open_m, sh.fetch_cap.size() == g.count() ? &sh.fetch_cap : nullptr, stride));
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
  // The wind, through the one place every node reads it (scene_wind.h; O14's scene wind).
  const SceneWind w =
      scene_wind(app, n, bind_prefix, kWaterWindKeys, {s.wind_speed, s.wind_dir, s.gustiness});
  s.wind_speed = w.mean_speed(); // with a storm's gust front: what the sea grows toward (F4)
  s.wind_dir = w.mean_dir();
  s.gustiness = w.gustiness;
  s.gust_speed = w.speed; // the gusts' pattern drifts with the wind as authored
  s.gust_dir = w.dir;
  s.fetch = std::max(key("water.fetch", "fetch", s.fetch), 400.0);
  s.lee = key("water.lee", "lee", 1.0) != 0.0;
  s.sea_lag = std::max(key("water.sea_lag", "sea_lag", s.sea_lag), 0.0);
  s.sea_fade = std::max(key("water.sea_fade", "sea_fade", s.sea_fade), 0.0);
  s.current = std::max(key("water.current", "current", s.current), 0.0);
  s.chop = key("water.chop", "chop", s.chop);
  s.wave_amp = key("water.wave_amp", "wave_amp", s.wave_amp);
  s.shore_fade = std::max(key("water.shore_fade", "shore_fade", s.shore_fade), 0.05);
  s.swell_height = std::max(key("water.swell_height", "swell_height", s.swell_height), 0.0);
  s.swell_period = std::max(key("water.swell_period", "swell_period", s.swell_period), 3.0);
  s.swell_dir = key("water.swell_dir", "swell_dir", s.swell_dir);
  // C5 (D6): the bodies' waves pushing each other, off unless asked for. The window's cells are
  // a power of two (the nearest in 8 .. 128), its cell 5 cm .. 100 m, its depth at most 10 km (the
  // wake's limits, cvc::physics::field_wake).
  s.wake_push = key("water.wake_push", "wake_push", 0.0) != 0.0;
  const double cells = std::clamp(n.props.num("wake_push_cells", 64.0), 8.0, 128.0);
  s.wake_push_cells = 8;
  while (s.wake_push_cells < 128 && cells > 1.5 * s.wake_push_cells)
    s.wake_push_cells *= 2;
  s.wake_push_cell = std::clamp(n.props.num("wake_push_cell", s.wake_push_cell), 0.05, 100.0);
  s.wake_push_depth = std::min(n.props.num("wake_push_depth", s.wake_push_depth), 1.0e4);
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
  // Each sample's most fetch: an enclosed body's open extent; the sea's is the open fetch (-1).
  out->fetch_cap.assign(wb.body.size(), -1.0f);
  for (std::size_t i = 0; i < wb.body.size(); ++i) {
    const std::int32_t b = wb.body[i];
    if (b >= 0 && std::size_t(b) < wb.bodies.size() &&
        wb.bodies[std::size_t(b)].kind != cvc::world::water_kind::sea)
      out->fetch_cap[i] = float(wb.bodies[std::size_t(b)].fetch_m);
  }
  if (s.swell_height > 0.0) {
    const cvc::ocean::sea_state sea = sea_state_of(s);
    cvc::ocean::shore_swell sw;
    sw.hs_m = sea.swell.hs_m;
    sw.tp_s = sea.swell.tp_s;
    sw.dir_rad = sea.swell.dir_rad;
    cvc::ocean::shore_layer_params lp;
    lp.trains = 3; // the drawn water sums three
    lp.seed = kSwellSeed;
    lp.lee_exposure = kSwellLee;
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

// ── the sea as the wind grows it (O13) ─────────────────────────────────────────────────────────

double SceneWaterSea::weight(double t) const { return cvc::ocean::crossfade_weight(t, t0, t1); }

std::string format_sea_keyframe(const SceneWaterKeyframe &k) {
  char buf[512];
  std::snprintf(buf, sizeof(buf), "%llu %llu %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g",
                static_cast<unsigned long long>(k.keyframe),
                static_cast<unsigned long long>(k.from_tick), k.from.speed_mps, k.from.dir_rad,
                k.from.fetch_m, k.to.speed_mps, k.to.dir_rad, k.to.fetch_m, k.t0, k.t1);
  return buf;
}

bool parse_sea_keyframe(const std::string &text, SceneWaterKeyframe &out) {
  unsigned long long key = 0, from = 0;
  SceneWaterKeyframe k;
  char tail = 0;
  if (std::sscanf(text.c_str(), "%llu %llu %lf %lf %lf %lf %lf %lf %lf %lf %c", &key, &from,
                  &k.from.speed_mps, &k.from.dir_rad, &k.from.fetch_m, &k.to.speed_mps,
                  &k.to.dir_rad, &k.to.fetch_m, &k.t0, &k.t1, &tail) != 10 ||
      key == 0)
    return false;
  for (const double v : {k.from.speed_mps, k.from.dir_rad, k.from.fetch_m, k.to.speed_mps,
                         k.to.dir_rad, k.to.fetch_m, k.t0, k.t1})
    if (!std::isfinite(v))
      return false;
  k.keyframe = key;
  k.from_tick = from;
  out = k;
  return true;
}

bool scene_water_followed(cvc::app &app, const SceneNode &n, const std::string &bind_prefix,
                          SceneWaterKeyframe &out) {
  const std::string text =
      read_or<std::string>(app, resolve_bind(bind_prefix, node_key(n, "water.sea")), std::string());
  return !text.empty() && parse_sea_keyframe(text, out);
}

void scene_water_seed_follow(cvc::app &app, const SceneNode &n, const std::string &bind_prefix) {
  read_or_seed<std::string>(app, resolve_bind(bind_prefix, node_key(n, "water.sea")),
                            std::string());
  seed_shown_clock(app, bind_prefix);
}

double scene_water_time(cvc::app &app, const std::string &bind_prefix, double app_t) {
  return shown_time(app, bind_prefix, app_t);
}

bool SceneWaterKeyframe::operator==(const SceneWaterKeyframe &o) const {
  const auto same = [](const cvc::ocean::sea_wind &a, const cvc::ocean::sea_wind &b) {
    return a.speed_mps == b.speed_mps && a.dir_rad == b.dir_rad && a.fetch_m == b.fetch_m;
  };
  return same(from, o.from) && same(to, o.to) && t0 == o.t0 && t1 == o.t1 &&
         keyframe == o.keyframe && from_tick == o.from_tick;
}

struct SceneWaterSeaClock::impl {
  cvc::ocean::sea_lag lag;
  std::shared_ptr<const SceneWaterSea> sea;
  const SceneWaterShore *shore = nullptr; // the coast the fetch maps were built over
  bool lee = true;
};

SceneWaterSeaClock::SceneWaterSeaClock() : _impl(std::make_unique<impl>()) {}
SceneWaterSeaClock::~SceneWaterSeaClock() = default;
std::shared_ptr<const SceneWaterSea> SceneWaterSeaClock::sea() const { return _impl->sea; }

bool SceneWaterSeaClock::advance(double t, const SceneWaterSpec &s,
                                 const std::shared_ptr<const SceneWaterShore> &shore) {
  impl &d = *_impl;
  cvc::ocean::sea_lag_params lp = d.lag.params();
  lp.lag_s = s.sea_lag;
  lp.crossfade_s = s.sea_fade;
  d.lag.set_params(lp);
  const bool key = d.lag.advance(t, {s.wind_speed, s.wind_dir * kPi / 180.0, s.fetch});
  const bool coast = shore.get() != d.shore || s.lee != d.lee;
  if (!key && !coast && d.sea)
    return false;
  const auto t0 = std::chrono::steady_clock::now();
  auto next = std::make_shared<SceneWaterSea>(d.sea ? *d.sea : SceneWaterSea{});
  const SceneWaterSea *prev = d.sea.get();
  if (key) {
    next->from = d.lag.from();
    next->to = d.lag.to();
    next->t0 = d.lag.t0();
    next->t1 = d.lag.t1();
    // The old sea is the last keyframe's whole (a keyframe comes only once a crossfade is over).
    next->h0_from = prev ? prev->h0 : std::vector<cvc::ocean::h0_field>{};
    next->full_from = prev ? prev->full : keyframe_full(next->to);
    next->h0 = keyframe_h0(next->to);
    next->full = keyframe_full(next->to);
    next->fetch_from = prev ? prev->fetch : nullptr;
    ++next->keyframe;
  }
  // The fetch maps: the new keyframe's again when its direction or its open fetch moved (or the
  // coast or the lee switch changed); a change of speed alone keeps them. Both on the same coast.
  // So the map is always the last keyframe's wind's, as another host makes it from the keyframe
  // (follow, OW5q).
  if (!s.lee || !shore || !shore->grid) {
    next->fetch = next->fetch_from = nullptr;
  } else {
    const bool turned = !prev || !prev->fetch || next->to.dir_rad != prev->fetch->dir_rad ||
                        next->to.fetch_m != prev->fetch->open_m;
    if (coast || turned || !next->fetch)
      next->fetch = keyframe_fetch(*shore, next->to.dir_rad, next->to.fetch_m);
    if (coast || !next->fetch_from) {
      next->fetch_from =
          next->from.dir_rad == next->to.dir_rad && next->from.fetch_m == next->to.fetch_m
              ? next->fetch
              : keyframe_fetch(*shore, next->from.dir_rad, next->from.fetch_m);
    }
  }
  next->build_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  d.sea = std::move(next);
  d.shore = shore.get();
  d.lee = s.lee;
  return true;
}

bool SceneWaterSeaClock::follow(const SceneWaterKeyframe &k, const SceneWaterSpec &s,
                                const std::shared_ptr<const SceneWaterShore> &shore) {
  impl &d = *_impl;
  const bool coast = shore.get() != d.shore || s.lee != d.lee;
  const auto same = [](const cvc::ocean::sea_wind &a, const cvc::ocean::sea_wind &b) {
    return a.speed_mps == b.speed_mps && a.dir_rad == b.dir_rad && a.fetch_m == b.fetch_m;
  };
  if (k.keyframe == 0 || (!coast && d.sea && d.sea->keyframe == k.keyframe && d.sea->t0 == k.t0 &&
                          d.sea->t1 == k.t1 && same(d.sea->from, k.from) && same(d.sea->to, k.to)))
    return false; // the sea it has
  const auto t0 = std::chrono::steady_clock::now();
  auto next = std::make_shared<SceneWaterSea>();
  next->from = k.from;
  next->to = k.to;
  next->t0 = k.t0;
  next->t1 = k.t1;
  next->keyframe = k.keyframe;
  next->h0 = keyframe_h0(k.to);
  next->full = keyframe_full(k.to);
  // The old sea is the last keyframe's whole (advance: a keyframe comes once a crossfade is over),
  // the lag's `from`; the first keyframe has none.
  if (k.keyframe > 1) {
    next->h0_from = keyframe_h0(k.from);
    next->full_from = keyframe_full(k.from);
  } else {
    next->full_from = next->full;
  }
  if (s.lee && shore && shore->grid) {
    next->fetch = keyframe_fetch(*shore, k.to.dir_rad, k.to.fetch_m);
    if (k.keyframe > 1)
      next->fetch_from = k.from.dir_rad == k.to.dir_rad && k.from.fetch_m == k.to.fetch_m
                             ? next->fetch
                             : keyframe_fetch(*shore, k.from.dir_rad, k.from.fetch_m);
    else
      next->fetch_from = next->fetch; // advance's first keyframe: the coast's, both
  }
  next->build_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  d.sea = std::move(next);
  d.shore = shore.get();
  d.lee = s.lee;
  return true;
}

std::shared_ptr<const cvc::ocean::sea_forcing> scene_water_forcing(const SceneWaterSpec &s,
                                                                   const SceneWaterSea &k) {
  auto f = std::make_shared<cvc::ocean::sea_forcing>();
  if (!k.h0_from.empty()) {
    f->t0 = k.t0;
    f->t1 = k.t1;
  }
  f->fetch = k.fetch;
  f->fetch_from = k.fetch_from;
  f->wind = SceneWind{std::isnan(s.gust_speed) ? s.wind_speed : s.gust_speed,
                      std::isnan(s.gust_dir) ? s.wind_dir : s.gust_dir, s.gustiness}
                .state();
  f->current.speed_mps = s.current;
  return f;
}

std::vector<cvc::ocean::sea_cascade> scene_water_sea_cascades(const SceneWaterSpec &s,
                                                              const SceneWaterSea &k) {
  SceneWaterSpec ks = s; // the fades and decays for the new keyframe's wind
  ks.wind_speed = k.to.speed_mps;
  ks.wind_dir = k.to.dir_rad * 180.0 / kPi;
  ks.fetch = k.to.fetch_m;
  const SceneWaterCascades cd = scene_water_cascades(ks);
  std::vector<cvc::ocean::sea_cascade> cs;
  for (std::size_t i = 0; i < k.h0.size(); ++i) {
    cvc::ocean::sea_cascade c{k.h0[i], cd.fade_m[i], cd.decay_k[i]};
    if (i < k.h0_from.size())
      c.h0_from = k.h0_from[i];
    c.full_fetch_m = i < k.full.size() ? k.full[i] : 0.0;
    c.full_fetch_from_m = i < k.full_from.size() ? k.full_from[i] : c.full_fetch_m;
    c.gust = i < 3 ? kWaterGustWeight[i] : 0.0;
    c.advect = i < 3 && kWaterAdvect[i];
    cs.push_back(std::move(c));
  }
  return cs;
}

// ── the physics ──────────────────────────────────────────────────────────────────────────────

struct WaterRuntime::impl {
  cvc::app *app = nullptr;
  SceneNode node;
  std::string prefix;
  std::function<std::shared_ptr<const cvc::height_field>()> ground;
  std::shared_ptr<SceneColliders> colliders = std::make_shared<SceneColliders>();
  std::vector<double> built;    // the spec the field was built for
  std::vector<double> shoreSig; // and the coast
  SceneWaterSeaClock clock;     // the sea's keyframes of its lagged wind
  std::shared_ptr<const SceneWaterShore> shore;
  std::shared_ptr<const cvc::ocean::sea_surface> sea;
  std::shared_ptr<const cvc::physics::field> field;
  // With wake_push: the wake's mask from the coast (C6). It keeps its shore grid, so a new coast
  // never shares that grid's address.
  std::shared_ptr<const cvc::physics::shore_grid_mask> wakeShore;
  const float *groundData = nullptr; // the ground the field was built over (a regenerate swaps it)
  std::size_t groundSize = 0;
  // While the scene has storms (SceneColliders::clock_period): the clock period the sea last
  // stepped in (none yet: the largest value).
  std::uint64_t period_done = std::numeric_limits<std::uint64_t>::max();
  std::uint64_t sea_from_tick = 0; // the last keyframe's from_tick
  // A sea another host grew, followed in place of the lag (follow_sea, OW5q): the one to make
  // next, if new, and the last made.
  bool following = false;
  std::optional<SceneWaterKeyframe> follow_next;
  SceneWaterKeyframe followed;
};

WaterRuntime::WaterRuntime(cvc::app &app, const SceneNode &n, const std::string &bind_prefix,
                           std::function<std::shared_ptr<const cvc::height_field>()> ground)
    : _impl(std::make_unique<impl>()) {
  _impl->app = &app;
  _impl->node = n;
  _impl->prefix = bind_prefix;
  _impl->ground = std::move(ground);
  scene_water_seed_follow(app, n, bind_prefix);
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

std::shared_ptr<const SceneWaterSea> WaterRuntime::keyframes() const { return _impl->clock.sea(); }

SceneWaterKeyframe WaterRuntime::keyframe() const {
  const impl &d = *_impl;
  SceneWaterKeyframe k;
  if (const std::shared_ptr<const SceneWaterSea> s = d.clock.sea()) {
    k.from = s->from;
    k.to = s->to;
    k.t0 = s->t0;
    k.t1 = s->t1;
    k.keyframe = s->keyframe;
    k.from_tick = d.sea_from_tick;
  }
  return k;
}

void WaterRuntime::follow_sea(const SceneWaterKeyframe &k) {
  impl &d = *_impl;
  d.following = true;
  if (k != d.followed)
    d.follow_next = k;
}

bool WaterRuntime::following() const { return _impl->following; }

bool WaterRuntime::tick(const cvc::world_clock *on) {
  impl &d = *_impl;
  const std::shared_ptr<const cvc::height_field> g = d.ground ? d.ground() : nullptr;
  if (!g || g->rows() < 2 || g->cols() < 2)
    return false;
  const SceneWaterSpec s = scene_water_spec(*d.app, d.node, d.prefix);
  // The wind is not here: it reaches the field through the sea's keyframes (the clock below).
  const std::vector<double> spec{s.level,
                                 s.chop,
                                 s.wave_amp,
                                 s.shore_fade,
                                 s.swell_height,
                                 s.swell_period,
                                 s.swell_dir,
                                 s.outer,
                                 s.swash ? 1.0 : 0.0,
                                 s.lee ? 1.0 : 0.0,
                                 s.gustiness,
                                 s.current,
                                 s.wake_push ? 1.0 : 0.0,
                                 double(s.wake_push_cells),
                                 s.wake_push_cell,
                                 s.wake_push_depth};
  const std::vector<float> &heights = g->heights();
  const bool groundNew = heights.data() != d.groundData || heights.size() != d.groundSize;

  // The coast again when the ground, the level or the swell changed.
  const std::vector<double> shoreSig = scene_water_shore_signature(s);
  if (groundNew || shoreSig != d.shoreSig || !d.shore) {
    d.shore = scene_water_shore(*g, s);
    d.shoreSig = shoreSig;
  }
  // The sea follows its lagged wind: a keyframe (h0, and the fetch map when it turned) at most
  // once a crossfade, on the world's clock, the time physics and the renderer share. While the
  // scene has storms, whose fronts move the wind on their own, the sea steps once a clock period
  // (the scene's world sets the period): in period k, to the next period's start with the wind of
  // this one (wind.front holds the front at its start), and a keyframe's field goes in at the step
  // after that start (SceneColliders::from_tick). So where the sea's keyframes fall, and the step
  // the bodies feel each in, are the clock's alone, whatever the frames.
  bool seaNew = false;
  std::uint64_t from_tick = 0;
  const cvc::world_clock &clock = on ? *on : d.app->world_clock();
  if (SceneWaterKeyframe keyed; scene_water_followed(*d.app, d.node, d.prefix, keyed))
    follow_sea(keyed); // a host showing another host's sea (OW5q)
  if (d.following) {
    // Another host's sea (follow_sea, OW5q): its keyframe, made here as it was made there, and in
    // after the tick it went in after there. The coast again on this ground when it changed.
    const SceneWaterKeyframe k = d.follow_next ? *d.follow_next : d.followed;
    seaNew = d.clock.follow(k, s, d.shore);
    if (d.follow_next) {
      d.followed = *d.follow_next;
      d.follow_next.reset();
    }
    if (seaNew) {
      from_tick = k.from_tick;
      d.sea_from_tick = k.from_tick;
    }
  } else if (const double period = d.colliders->clock_period; period > 0.0 && d.clock.sea()) {
    const std::uint64_t ticks = scene_clock_period_ticks(period, clock.fixed_dt());
    const std::uint64_t k = clock.tick() / ticks;
    if (d.period_done != std::numeric_limits<std::uint64_t>::max() && k < d.period_done)
      d.period_done = k > 0 ? k - 1 : std::numeric_limits<std::uint64_t>::max(); // a restart
    if (d.period_done == std::numeric_limits<std::uint64_t>::max() || k > d.period_done) {
      const std::uint64_t next = (k + 1) * ticks;
      // A crossfade of at least two periods: a keyframe's field goes in (at `next`) before the next
      // keyframe can replace it, whether the runtime ticks before the world's steps (the world's
      // own, in its sync) or after them (a renderer's), so both put in the same fields.
      SceneWaterSpec sc = s;
      sc.sea_fade = std::max(s.sea_fade, 2.0 * static_cast<double>(ticks) * clock.fixed_dt());
      seaNew = d.clock.advance(static_cast<double>(next) * clock.fixed_dt(), sc, d.shore);
      d.period_done = k;
      if (seaNew) {
        from_tick = next;
        d.sea_from_tick = next;
      }
    }
  } else {
    seaNew = d.clock.advance(clock.t(), s, d.shore);
    if (seaNew)
      d.sea_from_tick = 0;
  }
  if (!groundNew && !seaNew && spec == d.built && d.colliders->geometry)
    return false;
  if (groundNew || spec != d.built || !d.colliders->geometry)
    from_tick = 0; // the ground or the keys changed: at once, as ever

  // The sea the renderer draws, on the CPU: the cascades fading at the shore and crossfading
  // between the keyframes, scaled by the fetch, the ripples by the gusts, the short ones carried by
  // the current; the swell's shore layer, and its swash on the beach, which the drawn node draws on
  // the ground (`swash: 0`: none).
  const std::shared_ptr<const SceneWaterSea> k = d.clock.sea();
  std::vector<cvc::ocean::sea_cascade> cs = scene_water_sea_cascades(s, *k);
  cvc::ocean::sea_params p;
  p.level = s.level;
  p.depth_m = 1000.0;
  p.wave_amp = s.wave_amp;
  p.chop = s.chop;
  p.source_cell_m = 0.5; // B3's source on a 0.5 m lattice, as lsystem_coast's debris
  p.swash = s.swash;
  d.sea = std::make_shared<const cvc::ocean::sea_surface>(
      std::move(cs), p, d.shore ? d.shore->grid : nullptr, d.shore ? d.shore->layer : nullptr,
      scene_water_forcing(s, *k));

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
  // A metre-sized body heaves with the water within a tenth of a second; sideways the water's
  // acceleration carries it on the waves and the current, and its drag against the water holds
  // it while a wind blows it through (field.h, inertial).
  item.field.damping = 1.0e4;
  item.field.inertial = true;
  item.field.surface = std::make_shared<cvc::physics::ocean_surface>(d.sea);
  // C5: the floating bodies' waves push the others (field.h, field_wake), when asked for, masked
  // from the coast (C6): they reflect off its steep shores, soak into its beaches and stay off the
  // land, a lake's or a pond's on all its banks; the drawn window's mask (shore_mask.h). The same
  // coast keeps the same mask (one digest), so a new keyframe of the sea keeps the waves.
  if (s.wake_push) {
    item.field.wake.cells = s.wake_push_cells;
    item.field.wake.cell = s.wake_push_cell;
    item.field.wake.depth = s.wake_push_depth;
    const cvc::ocean::shore_grid *grid = d.shore ? d.shore->grid.get() : nullptr;
    if (grid && grid->rows >= 2 && grid->cols >= 2 && grid->shore_m.size() == grid->count()) {
      if (!d.wakeShore || &d.wakeShore->grid() != grid)
        d.wakeShore = std::make_shared<const cvc::physics::shore_grid_mask>(
            d.shore->grid,
            d.shore->slope.size() == grid->count() ? d.shore->slope : std::vector<float>{},
            d.shore->layer);
      item.field.wake.shore = d.wakeShore;
    }
  }
  d.field = std::make_shared<const cvc::physics::field>(item.field);
  auto geo = std::make_shared<cvc::world::world_geometry>();
  geo->add_volume(std::move(item));
  d.colliders->geometry = geo;
  d.colliders->from_tick = from_tick;
  ++d.colliders->revision;
  d.built = spec;
  d.groundData = heights.data();
  d.groundSize = heights.size();
  return true;
}

} // namespace ariadne
} // namespace cvc
