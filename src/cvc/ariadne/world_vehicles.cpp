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
*/

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/data.h> // DataStore: a fleet's weights
#include <cvc/ariadne/scene.h> // SceneAsset
#include <cvc/ariadne/world_vehicles.h>
#include <cvc/core/app.h>
#include <cvc/core/process.h> // self_path
#include <cvc/core/sim_tick.h>
#include <cvc/geometry/height_field.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h> // training_scene: the trainer's vehicle (a fleet's metric)
#include <cvc/nav/dyn_4w.h> // dyn4_params_of
#include <cvc/nav/policy_library.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/world_grid.h>
#include <cvc/physics/dyn_4w_plant.h>
#include <cvc/physics/dyn_bike_plant.h>
#include <cvc/physics/dyn_boat_plant.h>
#include <cvc/physics/field.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/world.h>
#include <cvc/state/state.h>
#include <cvc/volume/volume.h>
#include <cvc/world/world_geometry.h>
#include <exception>
#include <filesystem>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <utility>

namespace cvc {
namespace ariadne {

namespace ph = cvc::physics;

namespace {

constexpr double kScale = 0.05;  // nav's normalized units per metre, as libcvc's nav demos
constexpr double kReach = 3.0;   // m from its goal that counts as there
constexpr double kSpacing = 8.0; // m kept between spawns drawn from the grid
constexpr double kPi = 3.14159265358979323846;
constexpr int kMaxSpawnCount = 256; // vehicles.spawn.count's most at one request
// How vehicles steer around each other (set_avoid): the separation's radius and strength in units
// of the largest vehicle's radius (cfg.veh.rr), and its pass-on-the-right share.
constexpr float kAvoidRadius = 5.0f;
constexpr float kAvoidGain = 1.5f;
constexpr float kAvoidSide = 0.6f;
constexpr float kAvoidSideRadius = 20.0f; // the turn's look-ahead, in vehicle radii

std::string num(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.6g", v);
  return buf;
}

std::string fixed(double v, int digits) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.*f", digits, v);
  return buf;
}

std::string rgb_text(const std::array<float, 3> &c) {
  return num(c[0]) + "," + num(c[1]) + "," + num(c[2]);
}

// HSV (h in [0, 1)) to RGB: a fleet's team colours, as the nav demos made them.
std::array<float, 3> hsv_rgb(double h, double s, double v) {
  const double i = std::floor(h * 6.0);
  const double f = h * 6.0 - i;
  const double p = v * (1.0 - s), q = v * (1.0 - f * s), t = v * (1.0 - (1.0 - f) * s);
  double r = v, g = t, b = p;
  switch (static_cast<int>(i) % 6) {
  case 1:
    r = q, g = v, b = p;
    break;
  case 2:
    r = p, g = v, b = t;
    break;
  case 3:
    r = p, g = q, b = v;
    break;
  case 4:
    r = t, g = p, b = v;
    break;
  case 5:
    r = v, g = p, b = q;
    break;
  default:
    break;
  }
  return {static_cast<float>(r), static_cast<float>(g), static_cast<float>(b)};
}

// splitmix64: a fleet's draws, the same on every platform.
std::uint64_t mix64(std::uint64_t &z) {
  std::uint64_t x = (z += 0x9e3779b97f4a7c15ull);
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ull;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebull;
  return x ^ (x >> 31);
}

// `n` free points of grid `on` (`dist` its distance field) at least `clearance` from anything
// blocked and, when they can be, `sep` from each other, drawn from `seed` among the samples whose
// column is in [c0, c1) (all of them when that band has none free): a fleet's starts and goals.
// Empty when nothing is free. Linear in n (the separation is checked in a hash of cells).
std::vector<std::array<double, 2>> band_points(const cvc::nav::world_grid &on,
                                               const std::vector<float> &dist, std::uint64_t seed,
                                               int n, double clearance, double sep, int c0,
                                               int c1) {
  std::vector<std::size_t> pool;
  const auto fill = [&](int a, int b, bool clear) {
    a = std::max(a, 0);
    b = std::min(b, on.cols);
    for (int r = 0; r < on.rows; ++r)
      for (int c = a; c < b; ++c) {
        const std::size_t i = static_cast<std::size_t>(r) * on.cols + c;
        if (i < dist.size() && (clear ? dist[i] >= clearance : dist[i] > 0.0f))
          pool.push_back(i);
      }
  };
  fill(c0, c1, true);
  if (pool.empty())
    fill(c0, c1, false);
  if (pool.empty())
    fill(0, on.cols, true);
  if (pool.empty())
    fill(0, on.cols, false);
  std::vector<std::array<double, 2>> pts;
  if (pool.empty() || n <= 0)
    return pts;
  const double h = on.cell();
  const double cell = std::max(sep, h);
  std::unordered_map<std::uint64_t, std::vector<std::size_t>> cells_of;
  const auto key = [&](double x, double y) {
    const auto ix = static_cast<std::int64_t>(std::floor((x - on.min_x) / cell));
    const auto iy = static_cast<std::int64_t>(std::floor((y - on.min_y) / cell));
    return (static_cast<std::uint64_t>(ix) << 32) ^ static_cast<std::uint32_t>(iy);
  };
  const auto crowded = [&](double x, double y) {
    if (!(sep > 0.0))
      return false;
    const auto ix = static_cast<std::int64_t>(std::floor((x - on.min_x) / cell));
    const auto iy = static_cast<std::int64_t>(std::floor((y - on.min_y) / cell));
    for (std::int64_t dx = -1; dx <= 1; ++dx)
      for (std::int64_t dy = -1; dy <= 1; ++dy) {
        const auto it = cells_of.find((static_cast<std::uint64_t>(ix + dx) << 32) ^
                                      static_cast<std::uint32_t>(iy + dy));
        if (it != cells_of.end())
          for (std::size_t k : it->second)
            if (std::hypot(pts[k][0] - x, pts[k][1] - y) < sep)
              return true;
      }
    return false;
  };
  std::uint64_t z = seed;
  pts.reserve(static_cast<std::size_t>(n));
  for (int k = 0; k < n; ++k) {
    std::array<double, 2> p{};
    for (int t = 0; t < 32; ++t) {
      const std::size_t i = pool[mix64(z) % pool.size()];
      p = {on.min_x + static_cast<double>(i % on.cols) * h,
           on.min_y + static_cast<double>(i / on.cols) * h};
      if (!crowded(p[0], p[1]))
        break;
    }
    cells_of[key(p[0], p[1])].push_back(pts.size());
    pts.push_back(p);
  }
  return pts;
}

// "r,g,b" in [0, 1]; false (and `out` unchanged) for anything else.
bool parse_rgb(const std::string &s, std::array<float, 3> &out) {
  std::array<float, 3> c{};
  std::stringstream in(s);
  std::string part;
  int n = 0;
  while (std::getline(in, part, ',')) {
    if (n >= 3)
      return false;
    try {
      c[static_cast<std::size_t>(n)] = std::clamp(std::stof(part), 0.0f, 1.0f);
    } catch (const std::exception &) {
      return false;
    }
    ++n;
  }
  if (n != 3)
    return false;
  out = c;
  return true;
}

// "key=value" pairs, separated by commas or new lines.
std::map<std::string, std::string> parse_metadata(const std::string &s) {
  std::map<std::string, std::string> out;
  std::string item;
  const auto take = [&](std::string t) {
    const auto trim = [](std::string x) {
      const std::size_t a = x.find_first_not_of(" \t\r");
      const std::size_t b = x.find_last_not_of(" \t\r");
      return a == std::string::npos ? std::string() : x.substr(a, b - a + 1);
    };
    const std::size_t eq = t.find('=');
    if (eq == std::string::npos)
      return;
    const std::string k = trim(t.substr(0, eq));
    if (cvc::state::isValidStateName(k))
      out[k] = trim(t.substr(eq + 1));
  };
  for (const char ch : s) {
    if (ch == ',' || ch == '\n') {
      take(item);
      item.clear();
    } else {
      item += ch;
    }
  }
  take(item);
  return out;
}

double clamp_to(const vehicle_archetype &a, const char *setting, double v) {
  const auto it = a.ranges.find(setting);
  return it == a.ranges.end() ? v : std::clamp(v, it->second.first, it->second.second);
}

// An intent's keys under `base` (`<base>.intent.*`, `<base>.force.*`), in this order.
constexpr const char *kIntentLeaves[] = {"intent.mode",   "intent.steer", "intent.accel",
                                         "intent.weight", "force.x",      "force.y",
                                         "force.steer"};
constexpr std::size_t kIntentKeys = sizeof(kIntentLeaves) / sizeof(kIntentLeaves[0]);
// What the keys read as when none is set: nothing to apply.
std::string no_intent_keys() { return std::string(kIntentKeys, '|'); }

// The grid's ground at a world point, bilinear between its corner samples (0 without heights).
// The dynamic models take the slope under each wheel by differences a few millimetres apart, so
// the ground must not step at cell edges as the nearest sample's does.
double grid_height(const cvc::nav::world_grid &g, double x, double y) {
  if (g.height.empty() || g.rows < 1 || g.cols < 1)
    return 0.0;
  const double cell = g.cell();
  if (!(cell > 0.0) || g.rows < 2 || g.cols < 2)
    return g.height.front();
  const double fx = std::clamp((x - g.min_x) / cell, 0.0, static_cast<double>(g.cols - 1));
  const double fy = std::clamp((y - g.min_y) / cell, 0.0, static_cast<double>(g.rows - 1));
  const int c0 = std::min(static_cast<int>(fx), g.cols - 2),
            r0 = std::min(static_cast<int>(fy), g.rows - 2);
  const double tx = fx - c0, ty = fy - r0;
  const auto h = [&](int r, int c) {
    return static_cast<double>(g.height[static_cast<std::size_t>(r) * g.cols + c]);
  };
  return (1.0 - ty) * ((1.0 - tx) * h(r0, c0) + tx * h(r0, c0 + 1)) +
         ty * ((1.0 - tx) * h(r0 + 1, c0) + tx * h(r0 + 1, c0 + 1));
}

// The ground a vehicle stands on at (x, y): the world's terrain where it has one (the physics
// collider's own heights), else the grid's heights between its samples. A start samples it under
// the whole chassis (nav_plant), so on a slope the vehicle starts on the ground, not partly in it.
double ground_at(const cvc::nav::world_grid *grid, const cvc::height_field *terrain, double x,
                 double y) {
  if (terrain) {
    const double h = terrain->height(x, y);
    if (std::isfinite(h))
      return h;
  }
  return grid ? grid_height(*grid, x, y) : 0.0;
}

// Whether two chassis boxes overlap in the plane (separating axes: each box's two).
bool boxes_overlap(const entity_info &a, const ph::vehicle_desc &da, const entity_info &b,
                   const ph::vehicle_desc &db) {
  const double ca = std::cos(a.yaw), sa = std::sin(a.yaw), cb = std::cos(b.yaw),
               sb = std::sin(b.yaw);
  const double ax[2][2] = {{ca, sa}, {-sa, ca}}, bx[2][2] = {{cb, sb}, {-sb, cb}};
  const double ha[2] = {da.half_extents.x, da.half_extents.y},
               hb[2] = {db.half_extents.x, db.half_extents.y};
  const double dx = b.x - a.x, dy = b.y - a.y;
  for (const auto *axes : {ax, bx})
    for (int k = 0; k < 2; ++k) {
      const double nx = axes[k][0], ny = axes[k][1];
      double ra = 0.0, rb = 0.0;
      for (int j = 0; j < 2; ++j) {
        ra += ha[j] * std::fabs(ax[j][0] * nx + ax[j][1] * ny);
        rb += hb[j] * std::fabs(bx[j][0] * nx + bx[j][1] * ny);
      }
      if (std::fabs(dx * nx + dy * ny) > ra + rb)
        return false;
    }
  return true;
}

bool finite_intent(const entity_intent &in) {
  return std::isfinite(in.steer) && std::isfinite(in.accel) && std::isfinite(in.force_x) &&
         std::isfinite(in.force_y) && in.weight >= 0.0 && in.weight <= 1.0;
}

// The intent the keys say: a value that is not a number (or a mode that is not one) counts as
// not given. With no mode, a steer or an acceleration replaces the command.
entity_intent intent_from_keys(const std::array<std::string, kIntentKeys> &v) {
  entity_intent in;
  const auto number = [](const std::string &t, double &out) {
    if (t.empty())
      return false;
    try {
      std::size_t used = 0;
      const double x = std::stod(t, &used);
      if (used != t.size() || !std::isfinite(x))
        return false;
      out = x;
      return true;
    } catch (const std::exception &) {
      return false;
    }
  };
  const bool steer = number(v[1], in.steer);
  const bool accel = number(v[2], in.accel);
  if (!parse_intent_mode(v[0], in.command))
    in.command = steer || accel ? entity_intent::mode::replace : entity_intent::mode::off;
  double w = in.weight;
  if (number(v[3], w))
    in.weight = std::clamp(w, 0.0, 1.0);
  number(v[4], in.force_x);
  number(v[5], in.force_y);
  in.force_steers = !(v[6] == "0" || v[6] == "false" || v[6] == "off");
  return in;
}

// A whole number of text (std::stod's, "nan" and "inf" included): false for anything else.
bool parse_number(const std::string &t, double &out) {
  if (t.empty())
    return false;
  try {
    std::size_t used = 0;
    const double x = std::stod(t, &used);
    if (used != t.size())
      return false;
    out = x;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

// A pose's numbers as a pose takes them: x, y, the angles, speed and steer finite; z and the time
// finite or not given (NaN), never infinite.
bool finite_pose(const entity_pose &p) {
  const auto given = [](double v) { return std::isfinite(v) || std::isnan(v); };
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.heading) &&
         std::isfinite(p.pitch) && std::isfinite(p.roll) && std::isfinite(p.speed) &&
         std::isfinite(p.steer) && given(p.z) && given(p.time);
}

// Two poses the same, number for number (NaN the same as NaN).
bool same_pose(const entity_pose &a, const entity_pose &b) {
  const auto eq = [](double u, double v) { return u == v || (std::isnan(u) && std::isnan(v)); };
  return eq(a.x, b.x) && eq(a.y, b.y) && eq(a.z, b.z) && eq(a.heading, b.heading) &&
         eq(a.pitch, b.pitch) && eq(a.roll, b.roll) && eq(a.speed, b.speed) &&
         eq(a.steer, b.steer) && eq(a.time, b.time) && a.rear_axle == b.rear_axle;
}

// The rotation Rz(heading) Ry(pitch) Rx(roll) as a quaternion (w, x, y, z).
ph::quat pose_quat(double heading, double pitch, double roll) {
  const double ch = std::cos(0.5 * heading), sh = std::sin(0.5 * heading);
  const double cp = std::cos(0.5 * pitch), sp = std::sin(0.5 * pitch);
  const double cr = std::cos(0.5 * roll), sr = std::sin(0.5 * roll);
  return {ch * cp * cr + sh * sp * sr, ch * cp * sr - sh * sp * cr, ch * sp * cr + sh * cp * sr,
          sh * cp * cr - ch * sp * sr};
}

// A row-major 4x4 matrix's translation and its rotation's heading, pitch and roll (R = Rz Ry Rx).
void pose_from_matrix(const double m[16], entity_pose &p) {
  p.x = m[3];
  p.y = m[7];
  p.z = m[11];
  p.pitch = std::asin(std::clamp(-m[8], -1.0, 1.0));
  p.heading = std::atan2(m[4], m[0]);
  p.roll = std::atan2(m[9], m[10]);
}

// "x,y,..." or "x y ...": the numbers, all of them, or false.
bool parse_numbers(const std::string &s, std::vector<double> &out) {
  out.clear();
  std::string tok;
  const auto take = [&]() {
    if (tok.empty())
      return true;
    double v = 0.0;
    if (!parse_number(tok, v))
      return false;
    out.push_back(v);
    tok.clear();
    return true;
  };
  for (const char ch : s) {
    if (ch == ',' || ch == ' ' || ch == '\t' || ch == '\n' || ch == '\r') {
      if (!take())
        return false;
    } else {
      tok += ch;
    }
  }
  return take();
}

// A pose's keys under `<base>.pose`, in this order.
constexpr const char *kPoseLeaves[] = {"x",     "y",     "z", "heading", "pitch", "roll",
                                       "speed", "steer", "t", "ref",     "matrix"};
constexpr std::size_t kPoseKeys = sizeof(kPoseLeaves) / sizeof(kPoseLeaves[0]);

// The pose the keys say: `matrix` (16 numbers, row-major) when it is one, else x and y (both
// needed) and the rest; a value that is not a number counts as not given. False: no pose.
bool pose_from_keys(const std::array<std::string, kPoseKeys> &v, entity_pose &out) {
  entity_pose p;
  std::vector<double> m;
  if (!v[10].empty() && parse_numbers(v[10], m) && m.size() == 16) {
    pose_from_matrix(m.data(), p);
  } else {
    if (!parse_number(v[0], p.x) || !parse_number(v[1], p.y))
      return false;
    parse_number(v[2], p.z);
    parse_number(v[3], p.heading);
    parse_number(v[4], p.pitch);
    parse_number(v[5], p.roll);
  }
  parse_number(v[6], p.speed);
  parse_number(v[7], p.steer);
  parse_number(v[8], p.time);
  p.rear_axle = v[9] == "rear" || v[9] == "rear_axle";
  if (!finite_pose(p))
    return false;
  out = p;
  return true;
}

} // namespace

std::string pack_poses(const std::vector<std::pair<std::string, entity_pose>> &records) {
  std::string out;
  char buf[32];
  const auto put = [&](double v) {
    if (std::isnan(v))
      out += " nan";
    else {
      std::snprintf(buf, sizeof(buf), " %.17g", v);
      out += buf;
    }
  };
  for (const auto &[key, p] : records) {
    if (!out.empty())
      out += '\n';
    out += key;
    for (const double v : {p.x, p.y, p.z, p.heading, p.pitch, p.roll, p.speed, p.steer, p.time})
      put(v);
    if (p.rear_axle)
      out += " rear";
  }
  return out;
}

bool parse_poses(const std::string &text, std::vector<std::pair<std::string, entity_pose>> &out) {
  out.clear();
  bool all = true;
  std::string record;
  const auto take = [&]() {
    std::vector<std::string> tok;
    std::string t;
    for (const char ch : record) {
      if (ch == ' ' || ch == '\t' || ch == ',' || ch == '\r') {
        if (!t.empty())
          tok.push_back(std::move(t));
        t.clear();
      } else {
        t += ch;
      }
    }
    if (!t.empty())
      tok.push_back(std::move(t));
    record.clear();
    if (tok.empty())
      return;
    entity_pose p;
    double *fields[9] = {&p.x,    &p.y,     &p.z,     &p.heading, &p.pitch,
                         &p.roll, &p.speed, &p.steer, &p.time};
    bool ok =
        tok.size() == 10 ||
        (tok.size() == 11 && (tok[10] == "rear" || tok[10] == "center" || tok[10] == "centre"));
    for (std::size_t i = 0; ok && i < 9; ++i)
      ok = parse_number(tok[i + 1], *fields[i]);
    p.rear_axle = tok.size() == 11 && tok[10] == "rear";
    if (ok && finite_pose(p))
      out.emplace_back(tok[0], p);
    else
      all = false;
  };
  for (const char ch : text) {
    if (ch == '\n' || ch == ';')
      take();
    else
      record += ch;
  }
  take();
  return all;
}

const char *intent_mode_name(entity_intent::mode m) {
  switch (m) {
  case entity_intent::mode::replace:
    return "replace";
  case entity_intent::mode::blend:
    return "blend";
  case entity_intent::mode::off:
    break;
  }
  return "off";
}

bool parse_intent_mode(const std::string &s, entity_intent::mode &out) {
  if (s == "off")
    out = entity_intent::mode::off;
  else if (s == "replace")
    out = entity_intent::mode::replace;
  else if (s == "blend")
    out = entity_intent::mode::blend;
  else
    return false;
  return true;
}

ph::dyn_boat_desc boat_desc_of(const vehicle_archetype &a, double mass, double max_speed) {
  const std::vector<std::string> presets = ph::dyn_boat_desc::presets();
  if (std::find(presets.begin(), presets.end(), a.preset) == presets.end())
    throw std::invalid_argument("vehicle '" + a.id + "': '" + a.preset + "' is not a boat preset");
  ph::dyn_boat_desc b = ph::dyn_boat_desc::preset(a.preset);
  for (const auto &[k, v] : a.overrides) {
    if (k == "mass")
      b.boat.hull.mass = v;
    else if (k == "max_speed")
      b.max_speed = v;
  }
  b.boat.hull.mass = clamp_to(a, "mass", mass > 0.0 ? mass : b.boat.hull.mass);
  b.max_speed = clamp_to(a, "max_speed", max_speed > 0.0 ? max_speed : b.max_speed);
  return b;
}

// A boat as a vehicle_desc, for what reads one (vehicle_desc_of).
static ph::vehicle_desc boat_stand_in(const ph::dyn_boat_desc &b) {
  ph::vehicle_desc d;
  double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
  for (const ph::vec3 &v : b.boat.hull.hull->vertices) {
    const double p[3] = {v.x, v.y, v.z};
    for (int k = 0; k < 3; ++k) {
      lo[k] = std::min(lo[k], p[k]);
      hi[k] = std::max(hi[k], p[k]);
    }
  }
  d.half_extents = {0.5 * (hi[0] - lo[0]), 0.5 * (hi[1] - lo[1]), 0.5 * (hi[2] - lo[2])};
  d.center_of_mass = {};
  d.mass = b.boat.hull.mass;
  d.max_speed = b.max_speed;
  d.max_accel = b.boat.propeller.max_thrust / d.mass;
  d.max_decel = b.boat.propeller.max_reverse_thrust / d.mass;
  d.max_steer = std::atan((hi[0] - lo[0]) / b.turn_radius);
  d.axles.clear();
  d.engine_max_torque = 0.0;
  return d;
}

double stand_height(const ph::vehicle_desc &d) {
  return d.half_extents.z + (d.axles.empty() ? 0.4 : d.axles.front().wheel_radius);
}

ph::vehicle_desc vehicle_desc_of(const vehicle_archetype &a, double mass, double max_speed) {
  if (vehicle_plant_floats(a.plant))
    return boat_stand_in(boat_desc_of(a, mass, max_speed));
  ph::vehicle_desc d;
  if (a.preset == "humvee")
    d = ph::vehicle_desc::humvee();
  else if (a.preset == "roboracer")
    d = ph::vehicle_desc::roboracer();
  else
    throw std::invalid_argument("vehicle '" + a.id + "': '" + a.preset + "' is not a preset");
  for (const auto &[k, v] : a.overrides) {
    if (k == "mass")
      d.mass = v;
    else if (k == "max_speed")
      d.max_speed = v;
    else if (k == "max_accel")
      d.max_accel = v;
    else if (k == "max_decel")
      d.max_decel = v;
    else if (k == "max_steer")
      d.max_steer = v;
  }
  d.mass = clamp_to(a, "mass", mass > 0.0 ? mass : d.mass);
  d.max_speed = clamp_to(a, "max_speed", max_speed > 0.0 ? max_speed : d.max_speed);
  return d;
}

std::string find_vehicle_model(const std::string &name) {
  namespace fs = std::filesystem;
  // A model's name, or its file's: "humvee" is Humvee.glb, and a name without an extension is
  // its .glb (TacticalBoat: TacticalBoat.glb).
  std::string file = name == "humvee" ? std::string("Humvee.glb") : name;
  if (!file.empty() && fs::path(file).extension().empty())
    file += ".glb";
  std::error_code ec;
  const auto is_file = [&](const fs::path &p) { return fs::is_regular_file(p, ec); };
  if (const char *b = std::getenv("CVC_NAV_BUNDLE"); b && *b) {
    const fs::path bundle(b);
    for (const fs::path &p : {bundle / file, bundle.parent_path() / "shared" / file})
      if (is_file(p))
        return p.string();
  }
  std::vector<fs::path> prefixes;
  for (const char *v : {"CVC_PREFIX", "CVCPKG_PREFIX", "CONDA_PREFIX"})
    if (const char *e = std::getenv(v); e && *e)
      prefixes.emplace_back(e);
  const fs::path self = cvc::process::self_path();
  if (!self.empty())
    prefixes.push_back(self.parent_path().parent_path());
  for (const fs::path &prefix : prefixes) {
    const fs::path scenes = prefix / "share" / "cvc-scenes";
    if (is_file(scenes / "shared" / file))
      return (scenes / "shared" / file).string();
    if (!fs::is_directory(scenes, ec))
      continue;
    std::vector<fs::path> dirs;
    for (const fs::directory_entry &d : fs::directory_iterator(scenes, ec))
      if (d.is_directory(ec))
        dirs.push_back(d.path());
    std::sort(dirs.begin(), dirs.end()); // the same pick on every platform
    for (const fs::path &d : dirs)
      if (is_file(d / file))
        return (d / file).string();
  }
  return std::string();
}

void resolve_vehicle_looks(std::vector<vehicle_archetype> &archetypes, const std::string &doc_dir,
                           const std::vector<SceneAsset> &assets,
                           std::vector<std::string> *warnings, const std::vector<SceneData> &data) {
  namespace fs = std::filesystem;
  const auto warn = [&](const std::string &w) {
    if (warnings)
      warnings->push_back(w);
  };
  const auto relative = [](const std::string &p) {
    return fs::path(p).is_relative() && p.find("://") == std::string::npos;
  };
  for (vehicle_archetype &v : archetypes) {
    vehicle_look &look = v.look;
    if (look.kind.empty()) { // the default: a preset's model, when it is installed
      const std::string model = v.preset == "humvee"     ? find_vehicle_model("humvee")
                                : v.preset == "tactical" ? find_vehicle_model("TacticalBoat")
                                                         : std::string();
      if (!model.empty()) {
        look.kind = "file";
        look.path = model;
        look.fit = true;
      } else {
        look.kind = "box";
      }
      continue;
    }
    if (look.kind == "model") {
      const std::string model = find_vehicle_model(look.path);
      if (model.empty()) {
        warn("world: vehicles: " + v.id + ": look: model '" + look.path +
             "' is not installed (cvcpkg install vehicle-" + look.path +
             "); it is drawn as its box");
        look = vehicle_look{};
        look.kind = "box";
      } else {
        look.kind = "file";
        look.path = model;
        look.fit = true;
      }
      continue;
    }
    if (look.kind == "asset") {
      const SceneAsset *asset = nullptr;
      for (const SceneAsset &a : assets)
        if (a.id == look.path)
          asset = &a;
      if (asset && !asset->source.source_file.empty()) {
        look.kind = "file";
        look.path = asset->source.source_file;
        if (!asset->source.base_dir.empty() && relative(look.path))
          look.path = (fs::path(asset->source.base_dir) / look.path).string();
      } else {
        warn("world: vehicles: " + v.id + ": look: no file asset '" + look.path +
             "' in the document's assets:; it is drawn as its box");
        look = vehicle_look{};
        look.kind = "box";
      }
      continue;
    }
    if (look.kind == "data") {
      if (!look.uri.empty()) // resolved already (a look a host passed on whole)
        continue;
      const SceneData *entry = nullptr;
      for (const SceneData &d : data)
        if (d.name == look.data)
          entry = &d;
      if (entry && !entry->uri.empty()) {
        look.uri = entry->uri;
        look.subpath = entry->path;
      } else {
        warn("world: vehicles: " + v.id + ": look: no data: entry '" + look.data +
             "' in the document; it is drawn as its box");
        look = vehicle_look{};
        look.kind = "box";
      }
      continue;
    }
    if (look.kind == "file" && !doc_dir.empty() && relative(look.path))
      look.path = (fs::path(doc_dir) / look.path).string();
  }
}

void resolve_vehicle_looks(std::vector<vehicle_archetype> &archetypes, const std::string &doc_dir,
                           const std::vector<SceneAsset> &assets,
                           std::vector<std::string> *warnings) {
  resolve_vehicle_looks(archetypes, doc_dir, assets, warnings, std::vector<SceneData>{});
}

vehicle_stats vehicle_stats_of(const ph::vehicle_desc &d) {
  vehicle_stats s;
  s.length = 2.0 * d.half_extents.x;
  s.width = 2.0 * d.half_extents.y;
  s.height = 2.0 * d.half_extents.z;
  s.mass = d.mass;
  s.top_speed = d.max_speed;
  s.wheelbase = d.wheelbase();
  // The peak of T(rpm) w over the curve's points (linear between them, so the peak of T w on a
  // segment can fall inside it: sample each segment).
  std::vector<std::pair<double, double>> curve = d.torque_curve;
  if (curve.empty())
    curve = {{0.0, 0.8}, {d.engine_max_rpm * 2.0 / 3.0, 1.0}, {d.engine_max_rpm, 0.8}};
  double peak = 0.0;
  for (std::size_t i = 0; i + 1 < curve.size(); ++i)
    for (int k = 0; k <= 16; ++k) {
      const double t = k / 16.0;
      const double rpm = curve[i].first + t * (curve[i + 1].first - curve[i].first);
      if (rpm > d.engine_max_rpm)
        continue;
      const double frac = curve[i].second + t * (curve[i + 1].second - curve[i].second);
      peak = std::max(peak, frac * d.engine_max_torque * rpm * 2.0 * kPi / 60.0);
    }
  s.power_kw = peak / 1000.0;
  return s;
}

struct WorldVehicles::impl {
  cvc::app *app = nullptr;
  ph::world *world = nullptr;
  std::vector<vehicle_archetype> archetypes;
  std::string vkey, ekey; // <prefix>.vehicles, <prefix>.entities ("" none)
  std::unique_ptr<WorldSpawns> spawns;
  cvc::nav::policy_library policies;

  std::shared_ptr<const cvc::nav::world_grid> grid;
  std::shared_ptr<const cvc::height_field> terrain; // the world on's ground, when it has one
  std::vector<float> sdf;
  std::vector<ph::body_id> ground;
  // Every fleet's navigator config: the largest and fastest vehicle any archetype may be (each
  // vehicle on a plant plans with its own envelope and size, nav_plant's plan_with_vehicle).
  cvc::nav::sim_world::config cfg;
  double tick = 0.0; // the navigator's tick asked for (s; set_tick); 0: the world's (cvc::sim_tick)
  bool ready = false;      // cfg is the grid's
  bool stepping = false;   // before_step() began a step that after_step() ends
  bool posing = false;     // and moved the `pose` entities in it
  int every = 1;           // physics steps per nav tick
  std::uint64_t steps = 0; // physics steps since the navigator was built
  // The clock tick the step being taken ends on (before_step(clock_tick)), and the clock ticks the
  // navigator ticks on (tick % every), once it has ticked on one.
  bool have_clock = false;
  std::uint64_t clock_tick = 0;
  bool have_phase = false;
  std::uint64_t phase = 0;
  std::uint64_t draws = 0; // goals drawn (each draw's seed)
  bool wander = true;      // nav@1 draws another goal at each one it reaches
  bool avoid = true;       // vehicles steer around each other (set_avoid)
  std::vector<std::array<double, 2>> movers; // what else they steer around (set_movers), m
  std::string sense = "known"; // what the navigators know: known or fog (set_sense)
  bool drive_tel = false;  // the navigators capture their drive (set_drive_telemetry)

  // A fleet: the entities on one plant, with a navigator of their own over the grid (a navigator
  // drives one plant). Their bodies are all in the one physics world, so they meet each other and
  // the world's bodies, and it steps them on its clock.
  struct fleet {
    std::string plant; // jolt, dynamic, dynamic4w or kinematic
    int group = -1;    // the fleet (start: fleet:) it navigates alone, -1 the plant's unnamed ones
    std::string sense; // known or fog: what it was made knowing
    // It plans with the trainer's vehicle (a fleet's metric: trained), else each plant's own; and
    // its vehicle radius, which its separation is sized by.
    bool trained = false;
    float rr = 0.0f;
    std::unique_ptr<cvc::nav::sim_world> sim;
    std::unique_ptr<ph::nav_plant> jolt;
    std::unique_ptr<ph::dyn_bike_plant> bike;
    std::unique_ptr<ph::dyn_4w_plant> four;
    std::unique_ptr<ph::dyn_boat_plant> boat;
    // kinematic: each agent's box (by its index), and the poses it moves between over the tick's
    // steps (x, y of its centre, z, yaw): where the bicycle was, and where the tick took it
    std::vector<ph::body_id> boxes;
    std::vector<std::array<double, 4>> from, to;
    int sub = 0; // steps of the tick so far
    // The drive's external force on each agent (normalized), for the tick: the drive's threads
    // read it while sim->step runs.
    std::vector<std::array<float, 2>> force;
    // A tick has run since its navigator began capturing its drive (drive_tel): plants() reads
    // the drive's telemetry.
    bool tel_fresh = false;
    // kinematic: vehicles came or went since kinematics() last ran. It runs once, before the
    // navigator next steps (only its step reads them), not at each: placing n is then linear.
    bool kin_stale = false;
  };
  // A navigator's key, "<plant>" or a named fleet's "<plant>/<fleet>" (fleet_group::key), as its
  // parts: fleets is searched by it without building the string. fleet_of() runs for each vehicle
  // many times a tick.
  struct nav_ref {
    std::string_view plant, fleet;
    bool named = false; // a named fleet's: "<plant>/<fleet>"
  };
  // Key `k` against `r`'s string, ordered as std::string orders them.
  static int compare_key(std::string_view k, const nav_ref &r) {
    const std::string_view parts[3] = {r.plant,
                                       r.named ? std::string_view("/") : std::string_view(),
                                       r.named ? r.fleet : std::string_view()};
    for (const std::string_view p : parts) {
      const std::size_t n = std::min(k.size(), p.size());
      if (const int c = k.compare(0, n, p, 0, n); c != 0)
        return c;
      if (n < p.size())
        return -1; // `k` is a prefix of `r`'s string
      k.remove_prefix(n);
    }
    return k.empty() ? 0 : 1;
  }
  struct nav_less {
    using is_transparent = void;
    bool operator()(const std::string &a, const std::string &b) const { return a < b; }
    bool operator()(const std::string &a, const nav_ref &b) const { return compare_key(a, b) < 0; }
    bool operator()(const nav_ref &a, const std::string &b) const { return compare_key(b, a) > 0; }
  };
  // By plant, made as entities need them; a named fleet's own by "<plant>/<fleet>".
  std::map<std::string, std::unique_ptr<fleet>, nav_less> fleets;

  struct entity {
    std::uint64_t id = 0;
    entity_spec spec;
    ph::vehicle_desc desc;
    std::string plant;                     // its spec's, else its archetype's
    std::uint64_t agent = 0;               // its fleet navigator's id (0: not placed)
    double sx = 0.0, sy = 0.0, syaw = 0.0; // where it starts (resolved at its first placing)
    bool started = false;
    double gx = 0.0, gy = 0.0; // its goal
    // Its intent keys as last read: its own, the intents table's by id, and by name.
    std::array<std::string, 3> seen_intent{no_intent_keys(), no_intent_keys(), no_intent_keys()};
    entity_intent applied;  // the intent the last navigator tick used
    double wind_fx = 0.0, wind_fy = 0.0; // the wind's drag on it at the last tick (N), set_wind
    std::string command_by; // who commanded it then: its controller, intent, blend, hold or gym
    // A gym's command for the next tick (command()); not finite: the controller's.
    double cmd_steer = std::numeric_limits<double>::quiet_NaN();
    double cmd_accel = std::numeric_limits<double>::quiet_NaN();
    int hits = 0; // contacts with the world's bodies since the navigator's last tick
    bool hit_vehicle = false;
    // The dynamic and kinematic plants' odometer (m), from its position at each navigator tick.
    double distance = 0.0, ox = 0.0, oy = 0.0;
    bool odometer = false;
    int group = -1;    // its fleet (start: fleet:) in `groups`, -1 none
    int member = -1;   // its index in that fleet (0 first)
    int rally = -1;    // its team's rally point in that fleet, -1 none
    bool hold = false; // its goal is held (a fleet's): it does not wander from it
    // A `pose` entity (its controller pose): no fleet, a kinematic box moved to its poses.
    bool posed = false;
    ph::body_id box; // its chassis, once it has a pose and the world has it
    bool has_pose = false;
    // Its poses, as its chassis's centre: the one it moves from and the one it moves to (the last
    // taken), the world tick that one was taken at (the start of the step) and the span it moves
    // over (s; 0: it is there).
    entity_pose from, to;
    std::uint64_t taken = 0;
    double span = 0.0;
    entity_pose drawn;   // where its chassis is at the end of the last step (z resolved)
    bool moving = false; // its chassis was moved over the last step (it has a velocity)
    double rolled = 0.0; // m its wheels have rolled, at its speed
    // Its pose keys as last read: its own, the poses table's by id, by name and by the entry it
    // was spawned from; and its last record in each packed value (its own table's, the poses
    // table's).
    std::array<std::string, 4> seen_pose;
    std::array<entity_pose, 2> seen_packed;
    std::array<bool, 2> have_packed{false, false};
    std::string table_key; // the poses table's entry it was spawned from ("" none)
    std::string table_ask; // and what that entry asked for (sync_pose_table)
  };
  std::vector<entity> ents;

  // The fleets (start: entries with fleet:): their vehicles laid out together from a seed, in
  // teams at rally points, published as one (vehicles.fleets.<name>.*).
  struct fleet_group {
    vehicle_start start;
    std::uint64_t seed = 0; // the layout's: the start's, one more at each reseed
    bool laid_out = false;
    std::vector<std::array<double, 2>> rally; // each team's point (world metres)
    std::vector<std::array<float, 3>> colors; // each team's colour
    std::vector<std::string> seen_rally;      // each team's "x|y" as last written or read
    std::string seen_restart, seen_reseed;
    std::string status;
    // Its navigator (FL1b): what it knows and how its vehicles share it (the keys switch them).
    std::string sense, belief, seen_sense, seen_belief;
    // Its policy: what is asked (trained, builtin, data:<entry> or a uri; the key switches it),
    // what drives (`model`, null: the built-in net) and that, said (`drives`). `loading` is the
    // data store's node of weights still on their way, `file` the .cvcnav in it.
    std::string policy, seen_policy, drives, loading, file, sha256;
    cvc::nav::policy_library::entry model;
    bool warned = false; // the fallback has been logged
    // Its plants (FL1c): its vehicles' in turn, or each team's (the key switches them), and the
    // archetype's for a team or vehicle they leave out. Its metric: trained, vehicle or policy
    // (follows the policy; the key switches it).
    fleet_plants plants;
    std::string base_plant, seen_plant;
    std::string metric = "policy", seen_metric;
    bool summary() const { return start.keys != "all"; }
    std::string key(const std::string &plant) const { return plant + "/" + start.fleet; } // nav_ref
  };
  std::vector<fleet_group> groups;
  std::vector<vehicle_start> starts; // every entry spawn_starts was given, in order (starts())
  // Each fleet's poses as the last publish left them, swapped in whole (std::atomic_store), so
  // poses() reads them from any thread without a lock, as sim_thread's snapshot is read.
  using pose_book = std::map<std::string, std::shared_ptr<const fleet_poses>>;
  std::shared_ptr<const pose_book> book;
  bool hold_placing = false; // spawn() leaves its entity unplaced (a fleet places it laid out)
  std::uint64_t next_id = 1;
  std::uint64_t rev = 0;
  std::string status;
  std::set<std::string> published; // the entity ids with keys

  // The request keys as last seen (a change is a request).
  std::string seen_spawn, seen_remove, seen_clear, seen_sel;

  // Intents (S1). `mu` guards `intents` and `live`, which set_intent() reaches from any thread;
  // the rest is the stepping thread's.
  mutable std::mutex mu;
  std::map<std::uint64_t, entity_intent> intents;
  std::set<std::uint64_t> live;
  std::string ikey; // the intents table (world: vehicles: intents:), "" none
  // Poses (the `pose` controller). `mu` guards `pending` (set_pose's, taken at the next step),
  // `taken` (each one's last pose, for pose()) and `posed_ids`.
  std::map<std::uint64_t, entity_pose> pending, taken;
  std::set<std::uint64_t> posed_ids;
  std::size_t posed_count = 0; // the `pose` entities (the stepping thread's)
  std::string pkey;            // the poses table (world: vehicles: poses:), "" none
  double pose_jump = 20.0;     // m: a pose farther from the last is taken at once
  std::string seen_own_packed, seen_table_packed; // the packed values as last read
  // The table's entries a spawn refused, by key: what they asked for then (not asked again until
  // it changes).
  std::map<std::string, std::string> table_refused;
  // The scene's wind (set_wind), for the plants that feel no field.
  std::shared_ptr<const ph::flow_velocity> wind;
  // The still water this runtime added for its boats (ensure_water), invalid when none.
  ph::field_id still_water;

  static void sample_force(void *user, int i, int, float, float, float *fx, float *fy) {
    const fleet *f = static_cast<const fleet *>(user);
    if (i >= 0 && static_cast<std::size_t>(i) < f->force.size()) {
      *fx = f->force[static_cast<std::size_t>(i)][0];
      *fy = f->force[static_cast<std::size_t>(i)][1];
    } else {
      *fx = *fy = 0.0f;
    }
  }

  // The navigator key of an entity's: its plant's, or its fleet's own.
  nav_ref nav_key(const entity &e) const {
    if (e.group >= 0 && static_cast<std::size_t>(e.group) < groups.size())
      return {e.plant, groups[static_cast<std::size_t>(e.group)].start.fleet, true};
    return {e.plant, {}, false};
  }
  fleet *fleet_of(const entity &e) const {
    if (e.posed) // a `pose` entity is no plant's
      return nullptr;
    const auto it = fleets.find(nav_key(e));
    return it == fleets.end() ? nullptr : it->second.get();
  }
  int index(const entity &e) const {
    const fleet *f = fleet_of(e);
    return f && f->sim && e.agent ? f->sim->index_of(e.agent) : -1;
  }
  double ground_height(double x, double y) const {
    return ground_at(grid.get(), terrain.get(), x, y);
  }

  // Where a placed entity's chassis is: its body, the body's pose and its speed along its heading.
  struct pose {
    ph::body_id chassis;
    ph::vec3 position;
    ph::quat rotation;
    double yaw = 0.0, speed = 0.0;
    double vx = 0.0, vy = 0.0; // the chassis's velocity in the world (m/s)
    double steer = 0.0;        // the steering's bicycle angle (rad, + left)
  };
  static double yaw_of(const ph::quat &q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }
  pose pose_of(const fleet &f, int i) const {
    pose p;
    if (f.jolt) {
      const ph::vehicle_state vs = world->vehicle_state(f.jolt->vehicle(i));
      p.chassis = vs.chassis;
      p.position = vs.body.position;
      p.rotation = vs.body.rotation;
      p.speed = vs.speed;
      p.vx = vs.body.linear_velocity.x;
      p.vy = vs.body.linear_velocity.y;
      p.steer = vs.steer;
    } else {
      if (f.bike) {
        p.chassis = f.bike->chassis(i);
        p.speed = f.bike->body(i).vx;
      } else if (f.four) {
        p.chassis = f.four->chassis(i);
        p.speed = f.four->body(i).vx;
      } else if (f.boat) {
        p.chassis = f.boat->hull(i);
        p.speed = f.boat->state(i).speed / kScale;
      } else {
        p.chassis = f.boxes.at(static_cast<std::size_t>(i));
        p.speed = f.sim->speeds_normalized()[i] / kScale;
      }
      if (p.chassis.valid()) {
        const ph::body_state s = world->state(p.chassis);
        p.position = s.position;
        p.rotation = s.rotation;
        p.vx = s.linear_velocity.x;
        p.vy = s.linear_velocity.y;
      }
      if (const cvc::nav::vehicle_plant *pl = f.sim->plant())
        p.steer = pl->state(i).steer;
      else if (const float *c = f.sim->plant_steer_command())
        p.steer = c[i];
    }
    p.yaw = yaw_of(p.rotation);
    return p;
  }

  // A placed `pose` entity's chassis, where the last step put it.
  pose posed_pose(const entity &e) const {
    pose p;
    p.chassis = e.box;
    const entity_pose &d = e.drawn;
    p.position = {d.x, d.y, d.z};
    p.rotation = pose_quat(d.heading, d.pitch, d.roll);
    p.yaw = d.heading;
    p.speed = d.speed;
    p.vx = d.speed * std::cos(d.heading);
    p.vy = d.speed * std::sin(d.heading);
    p.steer = d.steer;
    return p;
  }
  // Where any placed entity's chassis is (false: not placed).
  bool where(const entity &e, pose &out) const {
    if (e.posed) {
      if (!e.box.valid())
        return false;
      out = posed_pose(e);
      return true;
    }
    const fleet *f = fleet_of(e);
    const int i = index(e);
    if (!f || i < 0)
      return false;
    out = pose_of(*f, i);
    return true;
  }

  // The kinematic bicycle's pose as its box takes it: the box's centre (ahead of its rear axle by
  // the axle's offset), on the ground under it, facing its heading.
  std::array<double, 4> bicycle_box(const fleet &f, int i, const ph::vehicle_desc &d) const {
    const float *o = f.sim->positions_normalized();
    const double h = f.sim->headings()[i];
    const double rx = o[2 * i] / kScale + grid->center_x(),
                 ry = o[2 * i + 1] / kScale + grid->center_y();
    const double rear = rear_axle_x(d);
    const double cx = rx - rear * std::cos(h), cy = ry - rear * std::sin(h);
    return {cx, cy, ground_height(cx, cy) + stand_height(d), h};
  }
  static double rear_axle_x(const ph::vehicle_desc &d) {
    double rear = 0.0;
    bool have = false;
    for (const ph::axle_desc &a : d.axles)
      if (!a.steered && (!have || a.x < rear)) {
        rear = a.x;
        have = true;
      }
    return rear;
  }
  static ph::quat yaw_quat(double yaw) {
    return {std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw)};
  }

  // The kinematic fleet's per-agent kinematics, from its vehicles' (after an agent comes or goes);
  // on the trained metric, the trainer's vehicle's (cfg.veh) for all.
  void kinematics(fleet &f) {
    if (f.trained)
      return;
    const int n = f.sim->size();
    std::vector<float> vmax(static_cast<std::size_t>(n)), amax(vmax.size()), L(vmax.size());
    for (const entity &e : ents)
      if (fleet_of(e) == &f) {
        const int i = index(e);
        if (i < 0)
          continue;
        vmax[i] = static_cast<float>((e.desc.max_speed > 0.0 ? e.desc.max_speed : 30.0) * kScale);
        amax[i] = static_cast<float>(0.8 * e.desc.max_accel * kScale);
        L[i] = static_cast<float>(e.desc.wheelbase() * kScale);
      }
    if (n > 0)
      f.sim->set_vehicle_kinematics(vmax.data(), amax.data(), L.data(), n);
  }

  // What fleet `group`'s navigator knows (its sense:, else the world's), and how many belief
  // planes it shares (1; grouped: max(2, n / 64); private: none made, each vehicle its own).
  const std::string &sense_of(int group) const {
    return group >= 0 && !groups[static_cast<std::size_t>(group)].sense.empty()
               ? groups[static_cast<std::size_t>(group)].sense
               : sense;
  }
  int members(int group) const {
    int n = 0;
    for (const entity &e : ents)
      n += e.group == group ? 1 : 0;
    return n;
  }
  int planes_of(int group) const {
    if (group < 0 || groups[static_cast<std::size_t>(group)].belief != "grouped")
      return 1;
    return std::max(2, members(group) / 64);
  }
  // Whether fleet `group` plans with the trainer's vehicle (D9): its metric:, else (policy) while
  // weights drive it or are on their way, so their arrival does not make it again. On the built-in
  // net, its own or a fallback's, each plant plans with its own. Unnamed vehicles: their own.
  bool trained_metric(int group) const {
    if (group < 0)
      return false;
    const fleet_group &g = groups[static_cast<std::size_t>(group)];
    if (g.metric == "trained" || g.metric == "vehicle")
      return g.metric == "trained";
    return static_cast<bool>(g.model.model) || !g.loading.empty();
  }
  // Fleet member `e`'s plant: its team's, its turn in the fleet's list, else the archetype's.
  std::string plant_of(const entity &e) const {
    const fleet_group &g = groups[static_cast<std::size_t>(e.group)];
    const std::string p = g.plants.of(e.member, e.rally);
    return p.empty() ? g.base_plant : p;
  }

  // The navigator on `plant` (a named fleet's own, `group`), made the first time an entity needs
  // it.
  fleet &fleet_for(const std::string &plant, int group = -1) {
    std::unique_ptr<fleet> &slot =
        fleets[group >= 0 ? groups[static_cast<std::size_t>(group)].key(plant) : plant];
    if (slot)
      return *slot;
    auto f = std::make_unique<fleet>();
    f->plant = plant;
    f->group = group;
    f->sense = sense_of(group);
    // A boat's navigator plans on the water deep enough for the deepest boat, land blocked.
    const std::shared_ptr<const cvc::nav::world_grid> plan =
        vehicle_plant_floats(plant) ? boat_grid() : grid;
    // Under fog the navigator starts knowing nothing (an empty prior) and senses the grid.
    const bool fog = f->sense == "fog";
    const std::vector<std::uint8_t> unknown(fog ? plan->occupancy.size() : 0, 0);
    cvc::nav::sim_world::config c = cfg;
    c.freeze_sense = !fog; // known: the world's grid; fog: sensed
    // Its metric (D9): the trainer's vehicle, at the metric its weights learned in, as the demos
    // plan (SceneSwarm), and its plants plan with it too; else each plant's own size and limits.
    f->trained = trained_metric(group);
    if (f->trained) {
      const cvc::nav::training_scene ts;
      cvc::nav::veh_params v;
      v.rr = ts.rr;
      v.d_hat = ts.d_hat;
      v.vmax = ts.vmax;
      v.dt = c.veh.dt; // the hosts' tick, not the trainer's 0.06 s
      c.veh = v;
      if (avoid) {
        c.sep_radius = kAvoidRadius * v.rr;
        c.sep_gain = kAvoidGain * v.rr;
        c.sep_side_radius = kAvoidSideRadius * v.rr;
      }
    }
    f->rr = c.veh.rr;
    f->sim = std::make_unique<cvc::nav::sim_world>(
        c, plan->occupancy.data(), fog ? unknown.data() : plan->occupancy.data(),
        cvc::nav::coef_mlp::default_biased(), nullptr, nullptr, nullptr, 0);
    for (int p = planes_of(group); p > 1; --p)
      f->sim->add_plane();
    f->sim->set_capture_drive_telemetry(drive_tel);
    // Its kernels on the app's pool: without one each fans out by spawning and joining threads
    // per call, most of a tick for hundreds of vehicles (FLEETS_AND_NIGHT_ROADMAP.md FL0).
    if (app)
      f->sim->set_thread_pool(&app->computePool());
    ph::nav_plant_options o;
    o.step_world = false; // the world steps on the clock; the plant commands and reads
    o.plan_with_vehicle = !f->trained;
    o.ground_bodies = ground;
    std::shared_ptr<const cvc::nav::world_grid> g = grid;
    std::shared_ptr<const cvc::height_field> t = terrain;
    o.ground_height = [g, t](double x, double y) { return ground_at(g.get(), t.get(), x, y); };
    const std::vector<ph::vehicle_desc> first{ph::vehicle_desc::humvee()};
    if (plant == "jolt") {
      f->jolt = std::make_unique<ph::nav_plant>(*world, first, std::move(o));
      f->sim->set_plant(f->jolt.get());
    } else if (plant == "dynamic") {
      ph::dyn_bike_plant_options b;
      b.plant = o;
      b.physics = world;
      b.ground_bodies = ground;
      b.step_world = false;
      f->bike = std::make_unique<ph::dyn_bike_plant>(first, std::move(b));
      f->sim->set_plant(f->bike.get());
    } else if (plant == "dynamic4w") {
      ph::dyn_4w_plant_options b;
      b.plant = o;
      b.physics = world;
      b.ground_bodies = ground;
      b.step_world = false;
      f->four = std::make_unique<ph::dyn_4w_plant>(first, std::move(b));
      f->sim->set_plant(f->four.get());
    } else if (vehicle_plant_floats(plant)) {
      make_boat_fleet(*f, o);
    } // kinematic: nav's bicycle, and a kinematic box each in the world
    slot = std::move(f);
    return *slot;
  }

  // The boats' water deep enough for the deepest boat archetype's draft and keel clearance: the
  // ground their navigator plans on (WorldSpawns' medium grid).
  std::shared_ptr<const cvc::nav::world_grid> boat_grid() const {
    spawn_need n;
    n.medium = cvc::nav::plant_medium::water;
    n.draft = deepest_draft();
    return spawns->ground_for(n).grid;
  }
  double deepest_draft() const {
    double deepest = 0.0;
    for (const vehicle_archetype &a : archetypes)
      if (vehicle_plant_floats(a.plant)) {
        const ph::dyn_boat_desc b = boat_desc_of(a);
        deepest = std::max(deepest, ph::dyn_boat_plant::draft(b.boat, b.boat.hull.mass, 1000.0));
      }
    return deepest;
  }

  // A boat fleet: a dyn_boat_plant that the world steps (step_world off), on the world's water.
  void make_boat_fleet(fleet &f, const ph::nav_plant_options &o) {
    ensure_water();
    ph::dyn_boat_plant_options b;
    b.plant = o;
    b.physics = world;
    b.step_world = false;
    f.boat = std::make_unique<ph::dyn_boat_plant>(
        std::vector<ph::dyn_boat_desc>{ph::dyn_boat_desc::launch()}, std::move(b));
    f.sim->set_plant(f.boat.get());
  }

  // The water a boat floats on. The world's own (a scene's water node's field) where it covers the
  // grid's water; else a still field at the grid's water level over the grid, from below its lowest
  // ground, added once per grid and world (it needs the app for its volume; without one the boats
  // find no water and are refused).
  void ensure_water() {
    if (still_water.valid() || !grid || !grid->has_water() || !app)
      return;
    const double h = grid->cell();
    for (std::size_t i = 0; i < grid->water_depth.size(); ++i)
      if (grid->water_depth[i] > 0.5) {
        const double x = grid->min_x + static_cast<double>(i % grid->cols) * h,
                     y = grid->min_y + static_cast<double>(i / grid->cols) * h;
        if (world->water_at({x, y, grid->water_level - 0.25}).density > 0.0)
          return; // the world has water there
        break;
      }
    double lowest = grid->water_level;
    for (const float z : grid->height)
      lowest = std::min(lowest, static_cast<double>(z));
    auto box = std::make_shared<cvc::volume>(*app, cvc::dimension(2, 2, 2), cvc::Float,
                                             cvc::bounding_box(grid->min_x, grid->min_y,
                                                               lowest - 1.0, grid->max_x,
                                                               grid->max_y, grid->water_level));
    box->fill(1000.0); // kg/m^3
    ph::field f;
    f.kind = ph::field_kind::fluid;
    f.volume = box;
    f.drag_coefficient = 1.0;
    f.damping = 1.0e4;
    still_water = world->add_field(f);
  }
  void drop_still_water() {
    if (still_water.valid())
      try {
        world->remove_field(still_water);
      } catch (const std::exception &) {
      }
    still_water = ph::field_id{};
  }

  // Each archetype's need of its start (WorldSpawns::set_needs): its medium its plant's
  // (vehicle_plant::medium()), and a boat's draft (the deepest boat's: they share a navigator) and
  // half its hull's length.
  void update_needs() {
    std::map<std::string, spawn_need> needs;
    const double deepest = deepest_draft();
    for (const vehicle_archetype &a : archetypes) {
      spawn_need n;
      try {
        if (const cvc::nav::vehicle_plant *p = fleet_for(a.plant).sim->plant())
          n.medium = p->medium();
      } catch (const std::exception &) {
      }
      if (n.medium == cvc::nav::plant_medium::water) {
        n.draft = deepest;
        n.half_length = vehicle_desc_of(a).half_extents.x;
      }
      needs[a.id] = n;
    }
    spawns->set_needs(std::move(needs));
  }
  // The ground entity `e` starts and aims on: its archetype's medium's.
  WorldSpawns::start_ground ground_of(const entity &e) const {
    return spawns->ground_for(e.spec.archetype);
  }
  bool floats(const entity &e) const { return vehicle_plant_floats(e.plant); }

  // An entity out of its fleet: its vehicle (or box) out of the world.
  void remove_from_fleet(entity &e) {
    if (e.posed) {
      if (e.box.valid())
        try {
          world->remove_body(e.box);
        } catch (const std::exception &) {
        }
      e.box = ph::body_id{};
      return;
    }
    fleet *f = fleet_of(e);
    if (!f || !e.agent)
      return;
    const int i = index(e);
    if (!f->boxes.empty() && i >= 0) {
      try {
        world->remove_body(f->boxes[static_cast<std::size_t>(i)]);
      } catch (const std::exception &) {
      }
      f->boxes.erase(f->boxes.begin() + i);
      f->from.erase(f->from.begin() + i);
      f->to.erase(f->to.begin() + i);
    }
    f->sim->remove_agent(e.agent);
    e.agent = 0;
    if (f->plant == "kinematic")
      f->kin_stale = true;
  }

  // Each entity's intent keys: a change (from what they read last) is a new intent.
  void read_intent_keys() {
    cvc::state *r = root();
    if (!r || (ekey.empty() && ikey.empty()))
      return;
    for (entity &e : ents) {
      const std::string id = std::to_string(e.id);
      const std::string bases[3] = {
          ekey.empty() ? std::string() : ekey + cvc::state::SEPARATOR + id,
          ikey.empty() ? std::string() : ikey + cvc::state::SEPARATOR + id,
          ikey.empty() || !cvc::state::isValidStateName(e.spec.name)
              ? std::string()
              : ikey + cvc::state::SEPARATOR + e.spec.name};
      for (std::size_t k = 0; k < 3; ++k) {
        if (bases[k].empty())
          continue;
        cvc::state *base = r->findDescendant(bases[k]);
        std::array<std::string, kIntentKeys> v;
        std::string sig;
        for (std::size_t j = 0; j < kIntentKeys; ++j) {
          if (base)
            if (cvc::state *leaf = base->findDescendant(kIntentLeaves[j]))
              v[j] = leaf->value();
          sig += v[j] + "|";
        }
        if (sig == e.seen_intent[k])
          continue;
        e.seen_intent[k] = sig;
        const entity_intent in = intent_from_keys(v);
        std::lock_guard<std::mutex> lock(mu);
        intents[e.id] = in;
      }
    }
  }

  // A fleet's navigator tick, before it steps: each of its entities' intent as its command
  // (sim_world's override_command) and its force on the drive (ext_force); an external entity
  // with no intent brakes to a stop and holds.
  void apply_intents(fleet &f, const std::map<std::uint64_t, entity_intent> &now) {
    f.force.assign(static_cast<std::size_t>(f.sim->size()), {0.0f, 0.0f});
    const double dt = every * world->config().fixed_dt;
    const bool on_plant = f.sim->plant() != nullptr;
    bool pushed = false, steers = false;
    for (entity &e : ents) {
      if (fleet_of(e) != &f)
        continue;
      const int i = index(e);
      if (i < 0)
        continue;
      const auto it = now.find(e.id);
      e.applied = it == now.end() ? entity_intent{} : it->second;
      const entity_intent &in = e.applied;
      const bool external = e.spec.controller == "external";
      const float steer = static_cast<float>(in.steer);
      const float accel = static_cast<float>(in.accel * kScale);
      // The kinematic drive has no one command to blend with (it steers each substep): there a
      // blend replaces.
      if (in.command == entity_intent::mode::replace ||
          (in.command == entity_intent::mode::blend && !on_plant) ||
          (external && in.command != entity_intent::mode::off)) {
        f.sim->override_command(i, steer, accel);
        e.command_by = "intent";
      } else if (in.command == entity_intent::mode::blend) {
        f.sim->override_command(i, steer, accel, static_cast<float>(in.weight));
        e.command_by = "blend";
      } else if (external) {
        const double v = pose_of(f, i).speed;
        const double a_b = std::max(e.desc.max_decel, 0.1);
        f.sim->override_command(i, 0.0f,
                                static_cast<float>(std::clamp(-v / dt, -a_b, a_b) * kScale));
        e.command_by = "hold";
      } else {
        e.command_by = e.spec.controller;
      }
      if (in.force_x != 0.0 || in.force_y != 0.0) {
        f.force[static_cast<std::size_t>(i)] = {static_cast<float>(in.force_x * kScale),
                                                static_cast<float>(in.force_y * kScale)};
        pushed = true;
        steers = steers || in.force_steers;
      }
    }
    // The scene's wind on the plants that feel no field (set_wind): its share of the drag, over the
    // mass, added to the drive as an intent's force is. Its share is the drag in the moving air
    // less the drag in still air, which the dynamic models already charge (nav_plant's si.drag), so
    // a calm wind adds nothing. A jolt vehicle's aero drag reads the wind in physics, and so does a
    // boat: its hull is a dynamic body, which the wind's flow field already drags on in the air.
    if (wind && !f.jolt && !f.boat) {
      const double t = world->field_time();
      for (entity &e : ents) {
        if (fleet_of(e) != &f)
          continue;
        const int i = index(e);
        if (i < 0)
          continue;
        const pose p = pose_of(f, i);
        const ph::vec3 w = wind->velocity(p.position, t);
        const double vx = p.speed * std::cos(p.yaw), vy = p.speed * std::sin(p.yaw);
        const double ux = w.x - vx, uy = w.y - vy;
        const double u = std::sqrt(ux * ux + uy * uy), v = std::fabs(p.speed);
        const double half_cda =
            0.5 * e.desc.air_density * e.desc.drag_coefficient * e.desc.frontal_area;
        e.wind_fx = half_cda * (u * ux + v * vx);
        e.wind_fy = half_cda * (u * uy + v * vy);
        const double m = std::max(e.desc.mass, 1.0);
        if (!(e.wind_fx != 0.0 || e.wind_fy != 0.0))
          continue;
        std::array<float, 2> &fi = f.force[static_cast<std::size_t>(i)];
        fi[0] += static_cast<float>(e.wind_fx / m * kScale);
        fi[1] += static_cast<float>(e.wind_fy / m * kScale);
        pushed = true;
        steers = true;
      }
    } else {
      for (entity &e : ents)
        if (fleet_of(e) == &f)
          e.wind_fx = e.wind_fy = 0.0;
    }
    if (pushed) {
      cvc::nav::ext_force ef;
      ef.sample = &impl::sample_force;
      ef.user = &f;
      ef.steer = steers;
      f.sim->set_ext_force(ef);
    } else {
      f.sim->clear_ext_force(); // the plain drive, byte for byte
    }
  }

  // The navigator's config for the grid; the fleets are made again as entities need them.
  void build_navigator() {
    fleets.clear();
    ready = false;
    sdf.clear();
    steps = 0;
    if (!grid)
      return;
    sdf = cvc::nav::signed_distance(*grid);
    // The tick in whole physics steps: by default the world's (cvc::sim_tick of its step, the
    // whole number of steps nearest the policies' 0.06 s: 7 at 120 Hz, 4 at 60); one asked for
    // (set_tick) exactly on the simulation clock's quantum (sim_tick.h), else the nearest whole
    // number of a step that does not divide it, with a note in the status.
    const double fixed_dt = world->config().fixed_dt;
    const double want = tick > 0.0 ? tick : cvc::sim_tick(fixed_dt);
    every = std::max(1, static_cast<int>(std::lround(want / fixed_dt)));
    const double dt = every * fixed_dt;
    if (std::fabs(dt - want) > 1e-9)
      status = "the tick " + std::to_string(want) + " s is not a whole number of the physics " +
               "world's " + std::to_string(fixed_dt) + " s steps; it is " + std::to_string(dt) +
               " s";
    // The largest and fastest vehicle any archetype may be (each vehicle on a plant plans with
    // its own envelope and size, nav_plant's plan_with_vehicle).
    double length = 0.0, top = 0.0, wheelbase = 0.0, steer = 1e9, accel = 1e9;
    for (const vehicle_archetype &a : archetypes) {
      const ph::vehicle_desc d = vehicle_desc_of(a);
      const auto r = a.ranges.find("max_speed");
      length = std::max(length, 2.0 * d.half_extents.x);
      top = std::max(top, r != a.ranges.end() ? r->second.second : d.max_speed);
      wheelbase = std::max(wheelbase, d.wheelbase());
      steer = std::min(steer, d.max_steer);
      accel = std::min(accel, d.max_accel);
    }
    if (archetypes.empty()) {
      length = 4.84;
      top = 20.0;
      wheelbase = 3.3;
      steer = 0.45;
      accel = 2.5;
    }
    if (!(wheelbase > 0.0)) // boats alone: no wheels; their plant plans each one's bicycle
      wheelbase = 0.7 * length;
    if (!(top > 0.0))
      top = 30.0;
    const double k = std::max(0.1, length / 4.84);
    cfg = cvc::nav::sim_world::config{};
    cvc::nav::configure(cfg, *grid, kScale);
    cfg.range_m = 60.0 * k;
    cfg.n_rays = 120;
    cfg.veh.rr = static_cast<float>(1.2 * k * kScale);
    cfg.veh.d_hat = static_cast<float>(3.0 * k * kScale);
    cfg.veh.dt = static_cast<float>(dt);
    cfg.veh.vmax = static_cast<float>(top * kScale);
    cfg.veh.L = static_cast<float>(wheelbase * kScale);
    cfg.veh.delta_max = static_cast<float>(steer);
    cfg.veh.a_max = static_cast<float>(0.8 * accel * kScale);
    cfg.veh.a_lat_max = static_cast<float>(0.5 * 9.80665 * kScale);
    cfg.veh.k_steer = 0.8f;
    cfg.veh.nsub = 5;
    cfg.veh.allow_reverse = true;
    cfg.reach_tol = static_cast<float>(kReach * kScale);
    cfg.freeze_sense = sense != "fog"; // known: the world's grid; fog: sensed (set_sense)
    // nav@1 comes to rest at its goal and stays there: without these a Humvee 150 m out met its
    // goal at ~19 m/s and parked ~28 m past it (on the bicycle it drove on), so a gym's agents
    // never were all at their goals at once.
    cfg.stop_at_goal = true;
    cfg.park_hold = true;
    // Vehicles steer around each other: the nav demos' separation for the largest vehicle (the
    // other plants' vehicles join in as foreign agents at each tick, navigator_tick).
    if (avoid) {
      cfg.sep_radius = kAvoidRadius * cfg.veh.rr;
      cfg.sep_gain = kAvoidGain * cfg.veh.rr;
      cfg.sep_side = kAvoidSide;
      cfg.sep_side_radius = kAvoidSideRadius * cfg.veh.rr;
    }
    ready = true;
  }

  // Whether a placed vehicle other than `self` is within r metres of (x, y).
  bool crowded(double x, double y, double r, const entity *self) const {
    for (const entity &o : ents) {
      pose p;
      if (&o == self || !where(o, p))
        continue;
      if (std::hypot(p.position.x - x, p.position.y - y) < r)
        return true;
    }
    return false;
  }

  // Put entity `e` in the world: at its start if that is free, else at a drawn point. "" or why.
  // A `pose` entity goes where its pose is, once it has one, with or without a grid.
  std::string place(entity &e) {
    if (e.posed)
      return place_posed(e);
    if (!ready)
      return "waiting for the world's grid";
    const double clearance = std::hypot(e.desc.half_extents.x, e.desc.half_extents.y) + 0.5;
    if (floats(e) && !ground_of(e).any())
      return "no water deep and open enough for " + e.spec.archetype;
    if (e.group >= 0 && static_cast<std::size_t>(e.group) < groups.size()) {
      if (!groups[static_cast<std::size_t>(e.group)].laid_out)
        lay_out(static_cast<std::size_t>(e.group));
      e.plant = plant_of(e); // its team's, now that the layout has given it one
    }
    if (!e.started) {
      double px = 0.0, py = 0.0, ph = 0.0;
      // A start off its medium (a car's in a lake, a boat's ashore) moves to a free point.
      if (e.spec.placed && free_at(e, e.spec.x, e.spec.y)) {
        e.sx = e.spec.x;
        e.sy = e.spec.y;
      } else if (!e.spec.placed && spawns &&
                 spawns->choose(
                     e.spec.team, e.spec.archetype,
                     [&](double x, double y, double r) { return crowded(x, y, r, &e); }, px, py,
                     ph)) {
        e.sx = px;
        e.sy = py;
        if (!std::isfinite(e.spec.yaw))
          e.spec.yaw = ph;
      } else {
        const std::array<double, 2> p = draw_point(clearance, &e);
        e.sx = p[0];
        e.sy = p[1];
      }
      e.syaw = e.spec.yaw;
      e.started = true;
    } else if (!free_at(e, e.sx, e.sy)) { // a new world where its start is blocked
      const std::array<double, 2> p = draw_point(clearance, &e);
      e.sx = p[0];
      e.sy = p[1];
    }
    if (e.hold) {
      // A fleet's: its goal is where its layout (or its rally point since) put it.
    } else if (e.spec.controller == "nav@1") {
      const std::array<double, 2> g = draw_point(clearance, &e);
      e.gx = g[0];
      e.gy = g[1];
    } else { // none or external: its goal is where it is, so it parks there and holds still
      e.gx = e.sx;
      e.gy = e.sy;
    }
    // Its heading as first placed (toward its first goal, when the spawn gave none), kept: a
    // restart puts it back as it started.
    if (!std::isfinite(e.syaw))
      e.syaw = e.spec.controller == "nav@1" ? std::atan2(e.gy - e.sy, e.gx - e.sx) : 0.0;
    cvc::nav::sim_world::agent_spec a;
    a.x = nx(e.sx);
    a.y = ny(e.sy);
    a.goal_x = nx(e.gx);
    a.goal_y = ny(e.gy);
    a.heading = static_cast<float>(e.syaw);
    std::copy(e.spec.color.begin(), e.spec.color.end(), a.color);
    std::copy(e.spec.color2.begin(), e.spec.color2.end(), a.color2);
    a.team = e.spec.team;
    a.name = e.spec.name;
    fleet *made = nullptr;
    try {
      fleet &f = fleet_for(e.plant, e.group);
      made = &f;
      if (f.jolt)
        a.vehicle = f.jolt->add_desc(e.desc);
      else if (f.bike)
        a.vehicle = f.bike->add_desc(e.desc);
      else if (f.four)
        a.vehicle = f.four->add_desc(e.desc);
      else if (f.boat)
        a.vehicle =
            f.boat->add_desc(boat_desc_of(*find(e.spec.archetype), e.desc.mass, e.desc.max_speed));
      if (e.spec.controller == "nav@1" && !e.spec.policy.empty()) {
        cvc::nav::policy_library::entry p =
            policies.get(cvc::nav::policy_ref{e.spec.policy, e.spec.policy_sha256});
        a.policy = p.model;
        a.policy_key = p.key;
      } else if (e.spec.controller == "nav@1" && e.group >= 0) { // its fleet's
        const fleet_group &g = groups[static_cast<std::size_t>(e.group)];
        a.policy = g.model.model;
        a.policy_key = g.model.model ? g.model.key : std::string();
      }
      if (e.group >= 0) { // its fleet's belief: its own, its team's group, or the one
        const fleet_group &g = groups[static_cast<std::size_t>(e.group)];
        if (g.belief == "private")
          a.own_plane = true;
        else if (const int n = f.sim->planes(); n > 1)
          a.plane = (e.rally >= 0 ? e.rally : std::max(0, e.member)) % n;
      }
      e.agent = f.sim->add_agent(a);
      const int i = f.sim->index_of(e.agent);
      for (const auto &[k, v] : e.spec.metadata)
        f.sim->set_metadata(i, k, v);
      if (f.plant == "kinematic") {
        // Its box: kinematic, moved where the bicycle goes, pushing what it meets.
        const std::array<double, 4> at = bicycle_box(f, i, e.desc);
        ph::body_desc b;
        b.shape = ph::box{e.desc.half_extents};
        b.motion = ph::motion_type::kinematic;
        b.position = {at[0], at[1], at[2]};
        b.rotation = yaw_quat(at[3]);
        b.allow_sleeping = false;
        f.boxes.insert(f.boxes.begin() + i, world->add_body(b));
        f.from.insert(f.from.begin() + i, at);
        f.to.insert(f.to.begin() + i, at);
        f.kin_stale = true;
      }
    } catch (const std::exception &ex) {
      if (made && e.agent)
        made->sim->remove_agent(e.agent);
      e.agent = 0;
      return ex.what();
    }
    e.odometer = false;
    return std::string();
  }

  // ---- `pose` entities ----

  // Where a chassis's centre stands at (x, y) on the ground, as a kinematic box does.
  double stand_z(const entity &e, double x, double y) const {
    return ground_height(x, y) + stand_height(e.desc);
  }

  // A pose given to `e`, from now: where it moves to over the steps from `now` (the world tick
  // at the start of the step it is taken at), as entity_pose says.
  void take_pose(entity &e, const entity_pose &given, std::uint64_t now) {
    {
      std::lock_guard<std::mutex> lock(mu);
      taken[e.id] = given;
    }
    entity_pose c = given;
    if (c.rear_axle) { // its chassis's centre, ahead of the rear axle along its heading
      const double rear = rear_axle_x(e.desc);
      c.x -= rear * std::cos(c.heading);
      c.y -= rear * std::sin(c.heading);
      c.rear_axle = false;
    }
    if (e.has_pose && std::isfinite(c.time) && std::isfinite(e.to.time)) {
      if (c.time == e.to.time)
        return; // the same pose again
      const entity_pose &at = e.box.valid() ? e.drawn : e.to;
      const double dz = std::isfinite(c.z) && std::isfinite(at.z) ? c.z - at.z : 0.0;
      if (c.time > e.to.time && std::sqrt((c.x - at.x) * (c.x - at.x) +
                                          (c.y - at.y) * (c.y - at.y) + dz * dz) <= pose_jump) {
        e.span = c.time - e.to.time; // from where it is now, over the two poses' span
        e.from = at;
        e.to = c;
        e.taken = now;
        return;
      }
    }
    e.from = e.to = c; // the first, an untimed one, a jump or a time gone back: there at once
    e.span = 0.0;
    e.taken = now;
    e.has_pose = true;
  }

  // Where `e` is at world tick `t`: between its two poses, or at the last; its z resolved.
  entity_pose target_at(const entity &e, std::uint64_t t) const {
    entity_pose p = e.to;
    if (e.span > 0.0 && t > e.taken) {
      const double elapsed = static_cast<double>(t - e.taken) * world->config().fixed_dt;
      const double s = std::clamp(elapsed / e.span, 0.0, 1.0);
      if (s < 1.0 - 1e-12) { // the span's end is the pose itself, not a rounding short of it
        const entity_pose &a = e.from, &b = e.to;
        const auto arc = [s](double u, double v) {
          return u + s * std::remainder(v - u, 2.0 * kPi);
        };
        p.x = a.x + s * (b.x - a.x);
        p.y = a.y + s * (b.y - a.y);
        p.z = std::isfinite(b.z) && std::isfinite(a.z) ? a.z + s * (b.z - a.z) : b.z;
        p.heading = arc(a.heading, b.heading);
        p.pitch = arc(a.pitch, b.pitch);
        p.roll = arc(a.roll, b.roll);
        p.speed = a.speed + s * (b.speed - a.speed);
        p.steer = a.steer + s * (b.steer - a.steer);
        p.time = std::isfinite(a.time) ? a.time + s * (b.time - a.time) : b.time;
      }
    }
    if (!std::isfinite(p.z))
      p.z = stand_z(e, p.x, p.y);
    return p;
  }

  // Its chassis into the world at `p`: a kinematic box, which pushes what it meets.
  void add_posed_body(entity &e, const entity_pose &p) {
    ph::body_desc b;
    b.shape = ph::box{e.desc.half_extents};
    b.motion = ph::motion_type::kinematic;
    b.position = {p.x, p.y, p.z};
    b.rotation = pose_quat(p.heading, p.pitch, p.roll);
    b.allow_sleeping = false;
    e.box = world->add_body(b);
    e.drawn = p;
    e.moving = false;
    ++rev;
  }

  std::string place_posed(entity &e) {
    if (!e.has_pose || e.box.valid())
      return std::string(); // it waits for its first pose
    try {
      add_posed_body(e, target_at(e, world->tick()));
    } catch (const std::exception &ex) {
      e.box = ph::body_id{};
      return ex.what();
    }
    return std::string();
  }

  // Its chassis to `p` by the end of this step: along the way (moved, so it pushes), at once for a
  // jump, and held exactly there, at rest, while the pose does not change.
  void move_posed(entity &e, const entity_pose &p, double dt) {
    const double dx = p.x - e.drawn.x, dy = p.y - e.drawn.y, dz = p.z - e.drawn.z;
    const ph::quat q = pose_quat(p.heading, p.pitch, p.roll);
    const bool still = dx == 0.0 && dy == 0.0 && dz == 0.0 && p.heading == e.drawn.heading &&
                       p.pitch == e.drawn.pitch && p.roll == e.drawn.roll;
    if (still && !e.moving) {
      // at rest where it is
    } else if (still || std::sqrt(dx * dx + dy * dy + dz * dz) > pose_jump) {
      ph::body_state s; // there now, with no velocity
      s.position = {p.x, p.y, p.z};
      s.rotation = q;
      world->set_state(e.box, s);
      e.moving = false;
    } else {
      world->move_kinematic(e.box, {p.x, p.y, p.z}, q);
      e.moving = true;
    }
    e.rolled += p.speed * dt;
    e.drawn = p;
  }

  entity *entity_by_id(std::uint64_t id) {
    for (entity &e : ents)
      if (e.id == id)
        return &e;
    return nullptr;
  }
  // The `pose` entity a poses table's key names: the one its entry spawned, else by id, else by
  // name.
  entity *posed_by_key(const std::string &key) {
    for (entity &e : ents)
      if (e.posed && e.table_key == key)
        return &e;
    for (entity &e : ents)
      if (e.posed && std::to_string(e.id) == key)
        return &e;
    for (entity &e : ents)
      if (e.posed && e.spec.name == key)
        return &e;
    return nullptr;
  }

  // A packed value's records (pack_poses) under `table`.poses, read when it changed: each record
  // that changed is a new pose for the entity it names (by id in the entities' own table, by
  // posed_by_key in the poses table).
  void read_packed(cvc::state &r, const std::string &table, std::string &seen, std::size_t slot,
                   std::uint64_t now) {
    if (table.empty())
      return;
    cvc::state *n = r.findDescendant(table + cvc::state::SEPARATOR + "poses");
    std::string text = n ? n->value() : std::string();
    if (text == seen)
      return;
    seen = std::move(text);
    std::vector<std::pair<std::string, entity_pose>> records;
    parse_poses(seen, records);
    for (const auto &[key, p] : records) {
      entity *e = nullptr;
      if (slot == 0) {
        for (entity &x : ents)
          if (x.posed && std::to_string(x.id) == key)
            e = &x;
      } else {
        e = posed_by_key(key);
      }
      if (!e || (e->have_packed[slot] && same_pose(e->seen_packed[slot], p)))
        continue;
      e->seen_packed[slot] = p;
      e->have_packed[slot] = true;
      take_pose(*e, p, now);
    }
  }

  // Each `pose` entity's pose keys (`<base>.pose.*`: its own, the poses table's by id, by name and
  // by its entry), then the packed values: a change (from what they read last) is a new pose.
  void read_pose_keys(std::uint64_t now) {
    cvc::state *r = root();
    if (!r || (ekey.empty() && pkey.empty()))
      return;
    for (entity &e : ents) {
      if (!e.posed)
        continue;
      const std::string id = std::to_string(e.id);
      const auto in_table = [&](const std::string &key) {
        return pkey.empty() || key.empty() ? std::string() : pkey + cvc::state::SEPARATOR + key;
      };
      const bool by_name = cvc::state::isValidStateName(e.spec.name) && e.spec.name != id;
      const bool by_entry =
          !e.table_key.empty() && e.table_key != id && (!by_name || e.table_key != e.spec.name);
      const std::string bases[4] = {ekey.empty() ? std::string()
                                                 : ekey + cvc::state::SEPARATOR + id,
                                    in_table(id), by_name ? in_table(e.spec.name) : std::string(),
                                    by_entry ? in_table(e.table_key) : std::string()};
      for (std::size_t k = 0; k < 4; ++k) {
        if (bases[k].empty())
          continue;
        cvc::state *base = r->findDescendant(bases[k] + cvc::state::SEPARATOR + "pose");
        std::array<std::string, kPoseKeys> v;
        std::string sig;
        if (base)
          for (std::size_t j = 0; j < kPoseKeys; ++j) {
            if (cvc::state *leaf = base->findDescendant(kPoseLeaves[j]))
              v[j] = leaf->value();
            sig += v[j] + "|";
          }
        if (sig == e.seen_pose[k])
          continue;
        e.seen_pose[k] = sig;
        entity_pose p;
        if (base && pose_from_keys(v, p))
          take_pose(e, p, now);
      }
    }
    read_packed(*r, ekey, seen_own_packed, 0, now);
    read_packed(*r, pkey, seen_table_packed, 1, now);
  }

  // Before each physics step: the poses given since the last (the keys, the packed values, then
  // set_pose's), and each `pose` entity's chassis on its way to where it is at the step's end (a
  // new one into the world at its first pose).
  void pose_step() {
    const double dt = world->config().fixed_dt;
    const std::uint64_t now = world->tick(), end = now + 1;
    read_pose_keys(now);
    std::map<std::uint64_t, entity_pose> given;
    {
      std::lock_guard<std::mutex> lock(mu);
      given.swap(pending);
    }
    for (const auto &[id, p] : given)
      if (entity *e = entity_by_id(id); e && e->posed)
        take_pose(*e, p, now);
    for (entity &e : ents) {
      if (!e.posed || !e.has_pose)
        continue;
      if (!ready) { // no navigator tick counts the contacts afresh: each step does
        e.hits = 0;
        e.hit_vehicle = false;
      }
      const entity_pose p = target_at(e, end);
      try {
        if (!e.box.valid())
          add_posed_body(e, p);
        else
          move_posed(e, p, dt);
      } catch (const std::exception &ex) {
        status = "entity " + std::to_string(e.id) + ": " + ex.what();
      }
    }
  }

  // The poses table's entries that name an archetype: each one's `pose` entity, spawned when it
  // first appears (or asks for another archetype or identity) and removed when it goes.
  void sync_pose_table(WorldVehicles &self) {
    cvc::state *r = root();
    if (!r || pkey.empty())
      return;
    std::map<std::string, entity_spec> want;
    std::map<std::string, std::string> asked; // each entry's request, as text
    if (cvc::state *t = r->findDescendant(pkey))
      for (const std::string &key : t->childNames()) {
        if (key == "poses")
          continue;
        cvc::state *c = t->findDescendant(key);
        const auto leaf = [&](const char *name) {
          cvc::state *l = c ? c->findDescendant(name) : nullptr;
          return l ? l->value() : std::string();
        };
        entity_spec s;
        s.archetype = leaf("archetype");
        if (s.archetype.empty())
          continue;
        s.controller = "pose";
        s.name = leaf("name");
        if (s.name.empty())
          s.name = key;
        const std::string team = leaf("team"), color = leaf("color"), color2 = leaf("color2");
        double n = 0.0;
        if (parse_number(team, n) && std::isfinite(n))
          s.team = static_cast<int>(n);
        parse_rgb(color, s.color);
        parse_rgb(color2, s.color2);
        asked[key] = s.archetype + "|" + s.name + "|" + team + "|" + color + "|" + color2;
        want[key] = std::move(s);
      }
    // Those whose entry went, or asks for something else now, go.
    std::vector<std::uint64_t> gone;
    for (const entity &e : ents)
      if (!e.table_key.empty()) {
        const auto it = asked.find(e.table_key);
        if (it == asked.end() || it->second != e.table_ask)
          gone.push_back(e.id);
      }
    for (const std::uint64_t id : gone)
      self.despawn(id);
    for (auto it = table_refused.begin(); it != table_refused.end();)
      it = asked.count(it->first) && asked[it->first] == it->second ? std::next(it)
                                                                    : table_refused.erase(it);
    for (auto &[key, spec] : want) {
      if (table_refused.count(key))
        continue;
      bool have = false;
      for (const entity &e : ents)
        have = have || e.table_key == key;
      if (have)
        continue;
      std::string why;
      const std::uint64_t id = self.spawn(spec, &why);
      if (!id) {
        table_refused[key] = asked[key];
        status = "poses table entry '" + key + "': " + why;
        continue;
      }
      entity *made = entity_by_id(id);
      made->table_key = key;
      made->table_ask = asked[key];
    }
  }

  // Every entity out of the navigators (its vehicle out of the world, unless the world is gone).
  void unplace_all(bool world_gone) {
    if (world_gone) { // the vehicles went with the old world: forget them, don't remove them
      for (auto &[name, f] : fleets) {
        if (f->bike)
          f->bike->world_gone();
        if (f->four)
          f->four->world_gone();
        if (f->boat)
          f->boat->world_gone();
        f->boxes.clear();
      }
      for (entity &e : ents) {
        e.agent = 0;
        e.box = ph::body_id{};
        e.from = e.to; // a `pose` entity goes back in at its last pose, at once
        e.span = 0.0;
      }
      fleets.clear();
      return;
    }
    for (entity &e : ents)
      remove_from_fleet(e);
  }

  void place_all() {
    for (entity &e : ents)
      if (!e.agent)
        if (std::string why = place(e); !why.empty() && ready)
          status = "entity " + std::to_string(e.id) + ": " + why;
    ++rev;
  }

  // Fleet `gi`'s layout: every vehicle's start and goal from its seed, its teams by their goals'
  // latitude (each team's goal held at its rally point) and their colours, as the nav demos'
  // swarms are made (sim_world's scatter_free, SceneSwarm's targets). Its vehicles are not placed
  // here; place() places each at its start.
  void lay_out(std::size_t gi) {
    fleet_group &g = groups[gi];
    g.laid_out = true;
    std::vector<entity *> mine;
    for (entity &e : ents)
      if (e.group == static_cast<int>(gi))
        mine.push_back(&e);
    if (mine.empty() || !grid)
      return;
    const entity &first = *mine.front();
    const WorldSpawns::start_ground gr = ground_of(first);
    const cvc::nav::world_grid &on = gr.grid ? *gr.grid : *grid;
    const std::vector<float> &dist = gr.sdf ? *gr.sdf : sdf;
    double clearance = std::hypot(first.desc.half_extents.x, first.desc.half_extents.y) + 0.5;
    if (floats(first))
      clearance = std::max(clearance, gr.clearance);
    const int n = static_cast<int>(mine.size());
    const bool opposed = g.start.layout == "opposed" && on.cols > 1;
    const int lo = opposed ? static_cast<int>(on.cols * 0.30) : on.cols;
    const int hi = opposed ? static_cast<int>(on.cols * 0.70) : 0;
    std::uint64_t z = g.seed;
    const std::uint64_t start_seed = mix64(z), goal_seed = mix64(z);
    std::vector<std::array<double, 2>> starts, goals;
    std::vector<double> headings(static_cast<std::size_t>(n),
                                 std::numeric_limits<double>::quiet_NaN());
    if (g.start.layout == "points" && spawns) {
      for (int k = 0; k < n; ++k) {
        double px = 0.0, py = 0.0, ph = 0.0;
        const auto taken = [&](double x, double y, double r) {
          for (const std::array<double, 2> &s : starts)
            if (std::hypot(s[0] - x, s[1] - y) < r)
              return true;
          return crowded(x, y, r, nullptr);
        };
        if (!spawns->choose(mine[static_cast<std::size_t>(k)]->spec.team, first.spec.archetype,
                            taken, px, py, ph))
          break;
        starts.push_back({px, py});
        headings[static_cast<std::size_t>(k)] = ph;
      }
      if (static_cast<int>(starts.size()) < n) { // too few points: the rest drawn
        const std::vector<std::array<double, 2>> more =
            band_points(on, dist, start_seed, n - static_cast<int>(starts.size()), clearance,
                        2.0 * clearance, 0, on.cols);
        starts.insert(starts.end(), more.begin(), more.end());
      }
    } else {
      starts = band_points(on, dist, start_seed, n, clearance, 2.0 * clearance, 0, lo);
    }
    goals = band_points(on, dist, goal_seed, n, clearance, 0.0, hi, on.cols);
    if (static_cast<int>(starts.size()) < n || static_cast<int>(goals.size()) < n) {
      g.status = "no free ground for the fleet";
      return; // each placed as a single vehicle is
    }
    // Teams: one rally point per latitude band, on the goal nearest the band's centre.
    const int T = g.start.rally;
    std::vector<int> team(static_cast<std::size_t>(n), -1);
    g.rally.clear();
    g.colors.clear();
    if (T > 0) {
      double ymin = std::numeric_limits<double>::infinity(), ymax = -ymin, xsum = 0.0;
      for (const std::array<double, 2> &q : goals) {
        ymin = std::min(ymin, q[1]);
        ymax = std::max(ymax, q[1]);
        xsum += q[0];
      }
      const double xmean = xsum / n;
      for (int t = 0; t < T; ++t) {
        g.colors.push_back(hsv_rgb(static_cast<double>(t) / T, 0.74, 0.98));
        const double ty = ymin + (t + 0.5) / T * std::max(1e-6, ymax - ymin);
        std::array<double, 2> best{xmean, ty};
        double bd = std::numeric_limits<double>::infinity();
        for (const std::array<double, 2> &q : goals)
          if (const double d = std::hypot(q[0] - xmean, q[1] - ty); d < bd) {
            bd = d;
            best = q;
          }
        g.rally.push_back(best);
      }
      for (int k = 0; k < n; ++k) {
        const double f =
            ymax > ymin ? (goals[static_cast<std::size_t>(k)][1] - ymin) / (ymax - ymin) : 0.0;
        team[static_cast<std::size_t>(k)] = std::clamp(static_cast<int>(f * T), 0, T - 1);
      }
    }
    const bool hsv = g.start.colors.empty() ? T > 0 : g.start.colors == "hsv";
    for (int k = 0; k < n; ++k) {
      entity &e = *mine[static_cast<std::size_t>(k)];
      const std::size_t ks = static_cast<std::size_t>(k);
      const int t = team[ks];
      e.sx = starts[ks][0];
      e.sy = starts[ks][1];
      if (!g.start.has_heading && std::isfinite(headings[ks]))
        e.spec.yaw = headings[ks];
      e.syaw = e.spec.yaw; // not finite: toward its goal (place())
      e.started = true;
      e.hold = true;
      e.rally = t;
      if (t >= 0) {
        e.spec.team = g.start.team + t;
        e.gx = g.rally[static_cast<std::size_t>(t)][0];
        e.gy = g.rally[static_cast<std::size_t>(t)][1];
      } else {
        e.gx = goals[ks][0];
        e.gy = goals[ks][1];
      }
      if (hsv)
        e.spec.color = t >= 0 ? g.colors[static_cast<std::size_t>(t)]
                              : hsv_rgb(static_cast<double>(k) / n, 0.74, 0.98);
      if (e.spec.controller != "nav@1") { // none or external: it parks where it starts
        e.gx = e.sx;
        e.gy = e.sy;
      }
    }
    g.status = "laid out: " + std::to_string(n) + " " + first.spec.archetype +
               (g.start.layout.empty() ? std::string(" scattered") : " " + g.start.layout) +
               ", seed " + std::to_string(g.seed) +
               (T > 0 ? ", " + std::to_string(T) + " rally points" : std::string());
    publish_rally(g);
  }

  // Fleet `g`'s rally keys: vehicles.fleets.<name>.rally.count and .<t>.{x,y,color}.
  void publish_rally(fleet_group &g) {
    g.seen_rally.assign(g.rally.size(), std::string());
    if (vkey.empty() || !root())
      return;
    const std::string at = vkey + ".fleets." + g.start.fleet + ".rally.";
    put(at + "count", std::to_string(g.rally.size()));
    for (std::size_t t = 0; t < g.rally.size(); ++t) {
      const std::string x = num(g.rally[t][0]), y = num(g.rally[t][1]);
      put(at + std::to_string(t) + ".x", x);
      put(at + std::to_string(t) + ".y", y);
      put(at + std::to_string(t) + ".color", rgb_text(g.colors[t]));
      g.seen_rally[t] = x + "|" + y;
    }
  }

  fleet_group *group_named(const std::string &name) {
    for (fleet_group &g : groups)
      if (g.start.fleet == name)
        return &g;
    return nullptr;
  }

  // Fleet `gi`'s rally point `t` to (x, y) (held to the grid): its team's goals move there.
  bool move_rally(std::size_t gi, int t, double x, double y) {
    fleet_group &g = groups[gi];
    if (t < 0 || static_cast<std::size_t>(t) >= g.rally.size() || !std::isfinite(x) ||
        !std::isfinite(y))
      return false;
    if (grid) {
      x = std::clamp(x, grid->min_x, grid->max_x);
      y = std::clamp(y, grid->min_y, grid->max_y);
    }
    g.rally[static_cast<std::size_t>(t)] = {x, y};
    for (entity &e : ents)
      if (e.group == static_cast<int>(gi) && e.rally == t && e.spec.controller == "nav@1") {
        e.gx = x;
        e.gy = y;
        if (fleet *f = fleet_of(e); f && index(e) >= 0)
          f->sim->retarget(index(e), nx(x), ny(y));
      }
    if (!vkey.empty() && root()) {
      const std::string at =
          vkey + ".fleets." + g.start.fleet + ".rally." + std::to_string(t) + ".";
      const std::string xs = num(x), ys = num(y);
      put(at + "x", xs);
      put(at + "y", ys);
      g.seen_rally[static_cast<std::size_t>(t)] = xs + "|" + ys;
    }
    ++rev;
    return true;
  }

  // Fleet `gi` laid out again (from its next seed when `reseed`): its vehicles out of the world
  // and back at their new starts.
  void restart_group(std::size_t gi, bool reseed) {
    fleet_group &g = groups[gi];
    if (reseed)
      ++g.seed;
    for (entity &e : ents)
      if (e.group == static_cast<int>(gi)) {
        remove_from_fleet(e);
        e.started = false;
        e.hold = false;
        e.rally = -1;
        e.spec.team = g.start.team;
        e.spec.yaw = g.start.has_heading ? g.start.heading * kPi / 180.0
                                         : std::numeric_limits<double>::quiet_NaN();
      }
    drop_navigators(gi);
    g.laid_out = false;
    if (ready)
      for (entity &e : ents)
        if (e.group == static_cast<int>(gi) && !e.agent)
          if (std::string why = place(e); !why.empty())
            status = "entity " + std::to_string(e.id) + ": " + why;
    ++rev;
  }

  // Fleet `gi`'s own navigators, once its vehicles have left them: made again as they are placed
  // (what they know, and their belief planes, from the start).
  void drop_navigators(std::size_t gi) {
    for (auto it = fleets.begin(); it != fleets.end();)
      it = it->second && it->second->group == static_cast<int>(gi) ? fleets.erase(it) : ++it;
  }

  // Fleet `gi` made again with what it knows now (sense, belief): its vehicles leave its
  // navigators and are placed again where they are, at rest, on new ones.
  void rebuild_navigators(std::size_t gi) {
    for (entity &e : ents)
      if (e.group == static_cast<int>(gi) && e.agent) {
        if (const fleet *f = fleet_of(e); f && index(e) >= 0) {
          const pose p = pose_of(*f, index(e));
          e.sx = p.position.x;
          e.sy = p.position.y;
          e.syaw = p.yaw;
        }
        remove_from_fleet(e);
      }
    drop_navigators(gi);
    if (ready)
      for (entity &e : ents)
        if (e.group == static_cast<int>(gi) && !e.agent)
          if (std::string why = place(e); !why.empty())
            status = "entity " + std::to_string(e.id) + ": " + why;
    ++rev;
  }
  // Fleet `gi`'s navigators made again when the metric they plan with is no longer its own (its
  // metric: or policy changed, or the weights fell back), its vehicles at rest where they are.
  void sync_metric(std::size_t gi) {
    const bool want = trained_metric(static_cast<int>(gi));
    for (const auto &[k, nav] : fleets)
      if (nav && nav->group == static_cast<int>(gi) && nav->trained != want) {
        rebuild_navigators(gi);
        return;
      }
  }

  // The policy fleet `gi` asks for (FL1b): trained, builtin, data:<entry> or a uri. Resolved now
  // where it can be (the built-in net, a file, an installed copy); a download or a data: entry
  // starts loading, the fleet driving on the built-in net until it arrives (poll_policy). Either
  // way `drives` says what drives, and a fallback says why, once in the host's log.
  static constexpr const char *kTrainedUri = "cvcpkg://cvc/grl-snam-weights";
  static constexpr const char *kTrainedPath = "share/grl-snam-weights/coef_sdf.cvcnav";
  void fall_back(fleet_group &g, const std::string &why) {
    g.model = {};
    g.loading.clear();
    g.drives = "builtin: no trained weights (" + why + ")";
    if (!g.warned) {
      g.warned = true;
      std::fprintf(stderr, "ari: world: vehicles: fleet '%s': %s\n", g.start.fleet.c_str(),
                   g.drives.c_str());
    }
  }
  void load_from_store(fleet_group &g, const std::string &uri, const std::string &path,
                       const std::string &sha256) {
    if (!app)
      throw std::runtime_error("no data store to fetch " + uri + " with");
    DataSpec spec;
    spec.uri = uri;
    const std::size_t slash = path.find_last_of('/');
    spec.path = slash == std::string::npos ? std::string() : path.substr(0, slash);
    g.file = slash == std::string::npos ? path : path.substr(slash + 1);
    g.sha256 = sha256;
    g.loading = DataStore::of(*app)->load(spec);
    g.drives = "builtin: loading " + uri;
  }
  void resolve_policy(std::size_t gi) {
    fleet_group &g = groups[gi];
    g.model = {};
    g.loading.clear();
    const std::string ask = g.policy.empty() ? "trained" : g.policy;
    try {
      if (ask == "builtin") {
        g.drives = "builtin";
      } else if (ask == "trained") {
        // $CVC_NAV_WEIGHTS, an installed copy, else the published package (settled, D8).
        // default_weights_path() names the baked path when nothing is there (so a load says
        // where it looked): only a file that is there is an installed copy.
        const char *env = std::getenv("CVC_NAV_WEIGHTS");
        std::string installed;
        if (env && *env) {
          installed = env;
        } else {
          const std::string found = cvc::nav::coef_mlp::default_weights_path();
          std::error_code ec;
          if (std::filesystem::is_regular_file(found, ec))
            installed = found;
        }
        if (!installed.empty()) {
          g.model = policies.get(cvc::nav::policy_ref{installed, ""});
          g.drives = "trained (" + installed + ", " + g.model.key + ")";
        } else {
          load_from_store(g, kTrainedUri, kTrainedPath, "");
        }
      } else if (ask.rfind("data:", 0) == 0) { // the document's data: entry
        const std::string entry = ask.substr(5);
        if (!app || vkey.size() <= 9)
          throw std::runtime_error("no document data: entry '" + entry + "'");
        const std::string doc = vkey.substr(0, vkey.size() - 9); // <prefix>.vehicles
        const std::string node = DataStore::of(*app)->resolve(doc + ".data." + entry);
        if (node.empty())
          throw std::runtime_error("no data: entry '" + entry + "'");
        DataStore::of(*app)->touch(node);
        g.loading = node;
        g.file = g.start.policy_path;
        g.sha256 = g.start.policy_sha256;
        g.drives = "builtin: loading data: " + entry;
      } else {
        const bool own = ask == g.start.policy; // the document's: its path: and sha256: hold
        const std::string path = own ? g.start.policy_path : std::string();
        const std::string sha = own ? g.start.policy_sha256 : std::string();
        if (ask.find("://") == std::string::npos || ask.rfind("file://", 0) == 0) {
          g.model = policies.get(cvc::nav::policy_ref{ask, sha});
          g.drives = "weights (" + ask + ", " + g.model.key + ")";
        } else {
          load_from_store(g, ask, path, sha);
        }
      }
    } catch (const std::exception &ex) {
      fall_back(g, ex.what());
    }
    apply_policy(gi);
    sync_metric(gi);
  }
  // Weights on their way: taken once the store has them, else the fallback says why.
  void poll_policy(std::size_t gi) {
    fleet_group &g = groups[gi];
    if (g.loading.empty() || !app)
      return;
    const std::string s = data_status(*app, g.loading);
    if (s != "ready" && s != "error" && !s.empty())
      return;
    const std::string node = g.loading;
    const std::string uri = g.policy.empty() || g.policy == "trained" ? kTrainedUri : g.policy;
    try {
      if (s != "ready") {
        cvc::state *err = cvc::state::instance(*app).findThroughLinks(node + ".error");
        throw std::runtime_error(err && !err->value().empty() ? err->value()
                                                              : "could not fetch " + uri);
      }
      std::string file = g.file;
      if (file.empty())
        for (const std::string &p : data_files(*app, node))
          if (p.size() > 7 && p.compare(p.size() - 7, 7, ".cvcnav") == 0) {
            file = p;
            break;
          }
      const DataBytes bytes = file.empty() ? nullptr : data_file(*app, node, file);
      if (!bytes)
        throw std::runtime_error(uri + " has no " + (file.empty() ? ".cvcnav" : file));
      g.model = policies.get_bytes(*bytes, g.sha256, uri);
      g.loading.clear();
      g.drives = std::string(g.policy.empty() || g.policy == "trained" ? "trained" : "weights") +
                 " (" + uri + ", " + g.model.key + ")";
    } catch (const std::exception &ex) {
      fall_back(g, ex.what());
    }
    apply_policy(gi);
    sync_metric(gi);
  }
  // Fleet `gi`'s vehicles on nav@1 drive with its policy now (a vehicle naming its own keeps it);
  // without a model, on the built-in net (policy 0). Weights the world can't run (a feature it
  // doesn't provide) fall back, said.
  void apply_policy(std::size_t gi) {
    fleet_group &g = groups[gi];
    try {
      for (const entity &e : ents) {
        if (e.group != static_cast<int>(gi) || e.spec.controller != "nav@1" ||
            !e.spec.policy.empty())
          continue;
        fleet *f = fleet_of(e);
        const int i = index(e);
        if (!f || i < 0)
          continue;
        if (g.model.model)
          f->sim->set_policy(i, g.model.model, g.model.key);
        else
          f->sim->reset_policy(i);
      }
    } catch (const std::exception &ex) {
      if (!g.model.model)
        throw;
      fall_back(g, ex.what());
      apply_policy(gi);
    }
  }

  // Each fleet's request and rally keys: a change is a request, a rally point's a move.
  void poll_groups() {
    for (std::size_t gi = 0; gi < groups.size(); ++gi) {
      fleet_group &g = groups[gi];
      const std::string at = "fleets." + g.start.fleet + ".";
      if (const std::string r = get(at + "restart.request"); r != g.seen_restart) {
        g.seen_restart = r;
        restart_group(gi, false);
      }
      if (const std::string r = get(at + "reseed.request"); r != g.seen_reseed) {
        g.seen_reseed = r;
        restart_group(gi, true);
      }
      if (const std::string p = get(at + "policy"); p != g.seen_policy) {
        g.seen_policy = p;
        if (p != g.policy) {
          g.policy = p;
          g.warned = false;
          resolve_policy(gi);
        }
      }
      poll_policy(gi);
      const std::string s = get(at + "sense"), b = get(at + "belief");
      if (s != g.seen_sense || b != g.seen_belief) {
        g.seen_sense = s;
        g.seen_belief = b;
        const bool sense_ok = s == "known" || s == "fog";
        const bool belief_ok = b == "shared" || b == "grouped" || b == "private";
        if ((sense_ok && s != g.sense) || (belief_ok && b != g.belief)) {
          if (sense_ok)
            g.sense = s;
          if (belief_ok)
            g.belief = b;
          rebuild_navigators(gi);
        }
      }
      // Its plants: its vehicles leave their navigators and start at rest on the new ones.
      if (const std::string p = get(at + "plant"); p != g.seen_plant) {
        g.seen_plant = p;
        fleet_plants want;
        const int teams = g.laid_out ? static_cast<int>(g.rally.size()) : g.start.rally;
        if (std::string why = parse_fleet_plants(p, want, teams); !why.empty()) {
          status = "fleet '" + g.start.fleet + "': plant: " + why;
        } else if (want != g.plants) {
          g.plants = std::move(want);
          rebuild_navigators(gi);
        }
      }
      if (const std::string m = get(at + "metric"); m != g.seen_metric) {
        g.seen_metric = m;
        if (fleet_metric_known(m) && m != g.metric) {
          g.metric = m;
          sync_metric(gi);
        }
      }
      for (std::size_t t = 0; t < g.rally.size() && t < g.seen_rally.size(); ++t) {
        const std::string xs = get(at + "rally." + std::to_string(t) + ".x"),
                          ys = get(at + "rally." + std::to_string(t) + ".y");
        if (xs + "|" + ys == g.seen_rally[t])
          continue;
        g.seen_rally[t] = xs + "|" + ys;
        char *end = nullptr;
        const double x = std::strtod(xs.c_str(), &end);
        const bool okx = end && end != xs.c_str();
        const double y = std::strtod(ys.c_str(), &end);
        if (okx && end && end != ys.c_str())
          move_rally(gi, static_cast<int>(t), x, y);
      }
    }
  }

  // Each fleet's poses (poses()) and its counts: vehicles.fleets.names, and
  // vehicles.fleets.<name>.{count, placed, arrived, teams, status, seed, tick, bounds, ...}.
  void publish_groups() {
    if (groups.empty()) {
      if (std::atomic_load(&book))
        std::atomic_store(&book, std::shared_ptr<const pose_book>());
      return;
    }
    const bool keys = !vkey.empty() && root();
    auto next = std::make_shared<pose_book>();
    const std::uint64_t tick = world->tick();
    std::string names;
    for (std::size_t gi = 0; gi < groups.size(); ++gi) {
      const fleet_group &g = groups[gi];
      names += (names.empty() ? "" : ",") + g.start.fleet;
      auto fp = std::make_shared<fleet_poses>();
      fp->tick = tick;
      int count = 0, placed = 0, arrived = 0;
      for (const entity &e : ents) {
        if (e.group != static_cast<int>(gi))
          continue;
        ++count;
        const fleet *f = fleet_of(e);
        const int i = index(e);
        if (!f || i < 0)
          continue;
        const pose p = pose_of(*f, i);
        if (std::hypot(e.gx - p.position.x, e.gy - p.position.y) < kReach + e.desc.half_extents.x)
          ++arrived;
        if (placed++ == 0) {
          fp->min_x = fp->max_x = p.position.x;
          fp->min_y = fp->max_y = p.position.y;
        }
        fp->min_x = std::min(fp->min_x, p.position.x);
        fp->max_x = std::max(fp->max_x, p.position.x);
        fp->min_y = std::min(fp->min_y, p.position.y);
        fp->max_y = std::max(fp->max_y, p.position.y);
        fp->id.push_back(e.id);
        fp->x.push_back(static_cast<float>(p.position.x));
        fp->y.push_back(static_cast<float>(p.position.y));
        fp->z.push_back(static_cast<float>(p.position.z));
        fp->yaw.push_back(static_cast<float>(p.yaw));
        fp->speed.push_back(static_cast<float>(p.speed));
        fp->team.push_back(static_cast<std::int16_t>(e.rally));
      }
      const fleet_poses &now = *fp;
      (*next)[g.start.fleet] = std::move(fp);
      if (!keys)
        continue;
      const std::string at = vkey + ".fleets." + g.start.fleet + ".";
      put(at + "tick", std::to_string(tick));
      put(at + "bounds", placed ? num(now.min_x) + "," + num(now.min_y) + "," + num(now.max_x) +
                                      "," + num(now.max_y)
                                : std::string());
      put(at + "count", std::to_string(count));
      put(at + "placed", std::to_string(placed));
      put(at + "arrived", std::to_string(arrived));
      put(at + "teams", std::to_string(g.rally.size()));
      put(at + "seed", std::to_string(g.seed));
      put(at + "status", g.status);
      put(at + "drives", g.drives);
      int planes = 0;
      for (const auto &[k, nav] : fleets)
        if (nav && nav->group == static_cast<int>(gi))
          planes += nav->sim->planes() - (g.belief == "private" ? 1 : 0); // plane 0 unused
      put(at + "planes", std::to_string(planes));
      put(at + "plans", trained_metric(static_cast<int>(gi)) ? "trained" : "vehicle");
    }
    if (keys)
      put(vkey + ".fleets.names", names);
    std::atomic_store(&book, std::shared_ptr<const pose_book>(std::move(next)));
  }

  // Each step of a tick: the dynamic plants' substep, the kinematic boxes a step further along the
  // bicycle's tick (the world then steps them all).
  void before_world_step() {
    for (auto &[name, f] : fleets) {
      if (f->sim->size() == 0)
        continue;
      if (f->bike)
        f->bike->before_world_step();
      else if (f->four)
        f->four->before_world_step();
      else if (f->boat)
        f->boat->before_world_step();
      else if (!f->jolt) {
        const double t = std::min(1.0, (f->sub + 1.0) / every);
        for (std::size_t i = 0; i < f->boxes.size(); ++i) {
          const std::array<double, 4> &a = f->from[i], &b = f->to[i];
          const double yaw = a[3] + t * std::remainder(b[3] - a[3], 2.0 * kPi);
          world->move_kinematic(
              f->boxes[i],
              {a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])},
              yaw_quat(yaw));
        }
        ++f->sub;
      }
    }
  }

  void after_world_step() {
    for (auto &[name, f] : fleets) {
      if (f->sim->size() == 0)
        continue;
      if (f->bike)
        f->bike->after_world_step();
      else if (f->four)
        f->four->after_world_step();
      else if (f->boat)
        f->boat->after_world_step();
    }
    count_contacts();
  }

  // The step's contacts, counted on the vehicles they touch: with the world's bodies other than
  // the ground, and with another vehicle. (A kinematic box meets no static or kinematic body: its
  // obstacles are its footprint on the grid, and another kinematic box an overlap; info() and
  // entities() see to those.)
  void count_contacts() {
    // The events first: a step with none that counts (a kinematic fleet's steps, its boxes
    // meeting none) looks no vehicle up.
    const std::vector<ph::contact_event> events = world->contacts();
    if (std::all_of(events.begin(), events.end(), [](const ph::contact_event &ev) {
          return ev.type == ph::contact_event::kind::removed;
        }))
      return;
    std::map<std::uint64_t, entity *> by_chassis;
    for (entity &e : ents)
      if (pose p; where(e, p) && p.chassis.valid())
        by_chassis[p.chassis.value] = &e;
    if (by_chassis.empty())
      return;
    const auto of = [&](ph::body_id b) -> entity * {
      const auto it = by_chassis.find(b.value);
      return it == by_chassis.end() ? nullptr : it->second;
    };
    for (const ph::contact_event &ev : events) {
      if (ev.type == ph::contact_event::kind::removed)
        continue;
      entity *a = of(ev.body_a), *b = of(ev.body_b);
      if (a && b) {
        a->hit_vehicle = b->hit_vehicle = true;
        continue;
      }
      entity *e = a ? a : b;
      if (!e)
        continue;
      const ph::body_id other = a ? ev.body_b : ev.body_a;
      if (std::find(ground.begin(), ground.end(), other) == ground.end())
        ++e->hits;
    }
  }

  // Whether a chassis box at (x, y, yaw) covers a blocked sample of the grid (its centre, its
  // corners and its edges' midpoints).
  bool footprint_blocked(double x, double y, double yaw, const ph::vehicle_desc &d) const {
    if (!grid)
      return false;
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double hx = d.half_extents.x, hy = d.half_extents.y;
    for (const double u : {-hx, 0.0, hx})
      for (const double v : {-hy, 0.0, hy})
        if (grid->occupied(x + u * c - v * s, y + u * s + v * c))
          return true;
    return false;
  }

  bool posed_placed() const {
    for (const entity &e : ents)
      if (e.posed && e.box.valid())
        return true;
    return false;
  }

  // The navigator's tick for every fleet: the intents, the step, the kinematic boxes' next poses,
  // nav@1's next goals, and the odometers.
  void navigator_tick() {
    read_intent_keys();
    std::map<std::uint64_t, entity_intent> now;
    {
      std::lock_guard<std::mutex> lock(mu);
      now = intents;
    }
    for (auto &[name, f] : fleets) {
      if (f->sim->size() == 0)
        continue;
      if (f->kin_stale) { // the vehicles that came or went since its last step
        kinematics(*f);
        f->kin_stale = false;
      }
      apply_intents(*f, now);
      // A command a gym gave for this tick, in place of the intent's or the controller's (once).
      for (entity &e : ents) {
        if (fleet_of(e) != f.get())
          continue;
        const int i = index(e);
        if (i >= 0 && std::isfinite(e.cmd_steer) && std::isfinite(e.cmd_accel)) {
          f->sim->override_command(i, static_cast<float>(e.cmd_steer),
                                   static_cast<float>(e.cmd_accel * kScale));
          e.command_by = "gym";
        }
      }
      if (f->plant == "kinematic")
        for (const entity &e : ents)
          if (fleet_of(e) == f.get())
            if (const int i = index(e); i >= 0)
              f->from[static_cast<std::size_t>(i)] = bicycle_box(*f, i, e.desc);
      // The other plants' vehicles, the `pose` entities and the world's other movers (a
      // holon's avatars), for this fleet's separation to steer around.
      if (avoid && (fleets.size() > 1 || posed_placed() || !movers.empty())) {
        std::vector<float> others;
        for (const std::array<double, 2> &m : movers) {
          others.push_back(nx(m[0]));
          others.push_back(ny(m[1]));
        }
        for (const entity &o : ents) {
          pose q;
          if (fleet_of(o) == f.get() || !where(o, q))
            continue;
          others.push_back(nx(q.position.x));
          others.push_back(ny(q.position.y));
        }
        f->sim->set_foreign_agents(std::move(others));
      } else if (!f->sim->foreign_agents().empty()) {
        f->sim->set_foreign_agents({});
      }
      f->sim->step(1);
      f->tel_fresh = f->sim->capture_drive_telemetry();
      if (f->plant == "kinematic") {
        for (const entity &e : ents)
          if (fleet_of(e) == f.get())
            if (const int i = index(e); i >= 0)
              f->to[static_cast<std::size_t>(i)] = bicycle_box(*f, i, e.desc);
        f->sub = 0;
      }
    }
    for (entity &e : ents) {
      // The gym's command was for this tick alone; the contacts count afresh.
      e.cmd_steer = e.cmd_accel = std::numeric_limits<double>::quiet_NaN();
      e.hits = 0;
      e.hit_vehicle = false;
      fleet *f = fleet_of(e);
      const int i = index(e);
      if (!f || i < 0)
        continue;
      const pose p = pose_of(*f, i);
      if (!f->jolt) { // the dynamic and kinematic plants keep no odometer of their own
        if (e.odometer)
          e.distance += std::hypot(p.position.x - e.ox, p.position.y - e.oy);
        e.ox = p.position.x;
        e.oy = p.position.y;
        e.odometer = true;
      }
      // nav@1 wanders: one at its goal draws another (a fleet's goal is held).
      if (e.spec.controller == "nav@1" && wander && !e.hold &&
          std::hypot(e.gx - p.position.x, e.gy - p.position.y) < kReach + e.desc.half_extents.x) {
        const double clearance = std::hypot(e.desc.half_extents.x, e.desc.half_extents.y) + 0.5;
        const std::array<double, 2> g = draw_point(clearance, &e);
        e.gx = g[0];
        e.gy = g[1];
        f->sim->retarget(i, nx(e.gx), ny(e.gy));
      }
    }
  }

  entity_info info(const entity &e) const {
    entity_info out;
    out.id = e.id;
    out.spec = e.spec;
    out.spec.mass = e.desc.mass;
    out.spec.max_speed = e.desc.max_speed;
    out.plant = e.plant;
    out.spec.plant = e.plant;
    out.goal_x = e.gx;
    out.goal_y = e.gy;
    out.reach = kReach + e.desc.half_extents.x;
    if (e.group >= 0 && static_cast<std::size_t>(e.group) < groups.size())
      out.fleet = groups[static_cast<std::size_t>(e.group)].start.fleet;
    out.rally = e.rally;
    if (e.posed)
      return posed_info(e, std::move(out));
    const fleet *f = fleet_of(e);
    const int i = index(e);
    if (!f || i < 0)
      return out;
    out.placed = true;
    const pose p = pose_of(*f, i);
    out.chassis = p.chassis;
    out.wind_fx = e.wind_fx;
    out.wind_fy = e.wind_fy;
    out.x = p.position.x;
    out.y = p.position.y;
    out.z = p.position.z;
    out.yaw = p.yaw;
    out.speed = p.speed;
    out.vx = p.vx;
    out.vy = p.vy;
    out.steer = p.steer;
    out.ground_z = ground_height(out.x, out.y);
    out.obstacle_contacts = e.hits;
    out.vehicle_contact = e.hit_vehicle;
    if (f->plant == "kinematic" && footprint_blocked(out.x, out.y, out.yaw, e.desc))
      out.obstacle_contacts = std::max(out.obstacle_contacts, 1);
    if (const float *s = f->sim->plant_steer_command())
      out.command_steer = s[i];
    if (const float *a = f->sim->plant_accel_command())
      out.command_accel = a[i] / kScale;
    const double tick = every * world->config().fixed_dt;
    if (f->jolt) {
      const ph::vehicle_state vs = world->vehicle_state(f->jolt->vehicle(i));
      out.power_kw = vs.energy.power / 1000.0;
      out.energy_wh = (vs.energy.source > 0.0 ? vs.energy.source : vs.energy.engine) / 3600.0;
      out.wh_per_km = vs.energy.wh_per_km();
      out.distance = vs.energy.distance;
      for (const ph::wheel_state &w : vs.wheels)
        out.wheel_load += w.load;
      out.aero_force =
          std::sqrt(vs.aero_force.x * vs.aero_force.x + vs.aero_force.y * vs.aero_force.y +
                    vs.aero_force.z * vs.aero_force.z);
      if (wind) { // its drag against the air, less the drag in still air: the wind's share
        const double half_cda =
            0.5 * e.desc.air_density * e.desc.drag_coefficient * e.desc.frontal_area;
        const double vx = p.speed * std::cos(p.yaw), vy = p.speed * std::sin(p.yaw);
        const double v = std::fabs(p.speed);
        out.wind_fx = vs.aero_force.x + half_cda * v * vx;
        out.wind_fy = vs.aero_force.y + half_cda * v * vy;
      }
      out.gear = vs.gear;
      out.rpm = vs.engine_rpm;
      out.throttle = vs.throttle;
      out.brake = vs.brake;
    } else {
      // The dynamic models' books: the drive's work (J) over the tick and since they began; the
      // kinematic bicycle keeps none. The wheels carry the vehicle's weight; throttle and brake
      // are the command's share of what the vehicle can do.
      double drive = 0.0, total = 0.0;
      if (f->bike) {
        drive = f->bike->last_tick(i).drive;
        total = f->bike->books(i).drive;
      } else if (f->four) {
        drive = f->four->last_tick(i).drive;
        total = f->four->books(i).drive;
      } else if (f->boat) {
        drive = f->boat->last_tick(i).drive;
        total = f->boat->books(i).drive;
      }
      out.power_kw = tick > 0.0 ? drive / tick / 1000.0 : 0.0;
      out.energy_wh = total / 3600.0;
      out.distance = e.distance;
      out.wh_per_km = out.distance > 1.0 ? out.energy_wh / (out.distance / 1000.0) : 0.0;
      out.wheel_load = e.desc.mass * 9.80665;
      if (f->sim->plant()) {
        out.throttle = std::clamp(out.command_accel / std::max(e.desc.max_accel, 1e-6), 0.0, 1.0);
        out.brake = std::clamp(-out.command_accel / std::max(e.desc.max_decel, 1e-6), 0.0, 1.0);
      }
    }
    out.reached = std::hypot(e.gx - out.x, e.gy - out.y) < out.reach;
    out.intent = e.applied;
    return out;
  }

  // A `pose` entity as it is: where the last step put it, at its pose's speed and steer, with
  // nothing driving it (no plant, no energy books; its wheels carry its weight).
  entity_info posed_info(const entity &e, entity_info out) const {
    out.posed = e.has_pose;
    out.pose = e.drawn;
    out.intent = e.applied;
    if (!e.box.valid()) {
      out.goal_x = out.goal_y = 0.0;
      return out;
    }
    out.placed = true;
    const pose p = posed_pose(e);
    out.chassis = p.chassis;
    out.x = p.position.x;
    out.y = p.position.y;
    out.z = p.position.z;
    out.yaw = p.yaw;
    out.speed = p.speed;
    out.vx = p.vx;
    out.vy = p.vy;
    out.steer = p.steer;
    out.goal_x = out.x; // it goes where it is put
    out.goal_y = out.y;
    out.ground_z = ground_height(out.x, out.y);
    out.obstacle_contacts = e.hits;
    out.vehicle_contact = e.hit_vehicle;
    if (footprint_blocked(out.x, out.y, out.yaw, e.desc)) // a kinematic box meets no building
      out.obstacle_contacts = std::max(out.obstacle_contacts, 1);
    out.wheel_load = e.desc.mass * 9.80665;
    return out;
  }

  // Its wheels in its chassis box's frame, as drawn.
  std::vector<entity_wheel> wheels(const entity &e) const {
    std::vector<entity_wheel> out;
    const fleet *f = fleet_of(e);
    const int i = index(e);
    pose p;
    if (!where(e, p))
      return out;
    // A world point into the box's frame: R^T (q - p).
    const ph::quat c{p.rotation.w, -p.rotation.x, -p.rotation.y, -p.rotation.z};
    const auto local = [&](const ph::vec3 &q) {
      const ph::vec3 v{q.x - p.position.x, q.y - p.position.y, q.z - p.position.z};
      // v' = c v c*, as two cross products
      const ph::vec3 u{c.x, c.y, c.z};
      const ph::vec3 t{2.0 * (u.y * v.z - u.z * v.y), 2.0 * (u.z * v.x - u.x * v.z),
                       2.0 * (u.x * v.y - u.y * v.x)};
      return ph::vec3{v.x + c.w * t.x + (u.y * t.z - u.z * t.y),
                      v.y + c.w * t.y + (u.z * t.x - u.x * t.z),
                      v.z + c.w * t.z + (u.x * t.y - u.y * t.x)};
    };
    if (f && f->four) {
      for (const ph::dyn_4w_plant::wheel_pose &w : f->four->wheels(i)) {
        const ph::vec3 l = local(w.center);
        out.push_back({l.x, l.y, l.z, w.steer, w.down});
      }
      return out;
    }
    // The other plants: each axle's two wheels at the desc's offsets; a Jolt wheel where its
    // contact puts it, the others on the ground under them, steered as the plant steers.
    const ph::vehicle_desc &d = e.desc;
    double steer = 0.0;
    std::vector<ph::wheel_state> jolt;
    if (!f) { // a `pose` entity: its pose's steer
      steer = e.drawn.steer;
    } else if (f->jolt) {
      jolt = world->vehicle_state(f->jolt->vehicle(i)).wheels;
    } else if (f->bike) {
      steer = f->bike->body(i).delta;
    } else if (const float *s = f->sim->plant_steer_command()) {
      steer = s[i];
    }
    const double cy = std::cos(p.yaw), sy = std::sin(p.yaw);
    std::size_t k = 0;
    for (const ph::axle_desc &ax : d.axles)
      for (const double side : {1.0, -1.0}) {
        entity_wheel w;
        w.x = ax.x;
        w.y = side * ax.half_track;
        w.steer = ax.steered ? steer : 0.0;
        if (e.posed && ax.wheel_radius > 0.0) // rolled at its pose's speed
          w.spin = std::remainder(e.rolled / ax.wheel_radius, 2.0 * kPi);
        const ph::vec3 at{p.position.x + w.x * cy - w.y * sy, p.position.y + w.x * sy + w.y * cy,
                          0.0};
        if (k < jolt.size()) {
          const ph::wheel_state &js = jolt[k];
          w.steer = js.steer;
          w.down = js.contact;
          if (js.contact) {
            const ph::vec3 l = local(
                {js.contact_point.x, js.contact_point.y, js.contact_point.z + ax.wheel_radius});
            w.x = l.x;
            w.y = l.y;
            w.z = l.z;
          } else {
            w.z = ax.z - ax.suspension_max;
          }
        } else {
          w.z = local({at.x, at.y, ground_height(at.x, at.y) + ax.wheel_radius}).z;
        }
        out.push_back(w);
        ++k;
      }
    return out;
  }

  cvc::state *root() const { return app ? &cvc::state::instance(*app) : nullptr; }
  std::string get(const std::string &leaf) const {
    cvc::state *r = root();
    if (!r || vkey.empty())
      return std::string();
    cvc::state *k = r->findDescendant(vkey + cvc::state::SEPARATOR + leaf);
    return k ? k->value() : std::string();
  }
  void put(const std::string &key, const std::string &value) {
    if (cvc::state *r = root())
      (*r)(key).value(value);
  }
  void seed(const std::string &key, const std::string &value) {
    cvc::state *r = root();
    if (!r)
      return;
    cvc::state *k = r->findDescendant(key);
    if (!k || k->value().empty())
      (*r)(key).value(value);
  }

  const vehicle_archetype *find(const std::string &id) const {
    for (const vehicle_archetype &a : archetypes)
      if (a.id == id)
        return &a;
    return nullptr;
  }

  // World metres <-> the navigator's normalized frame.
  float nx(double x) const { return static_cast<float>((x - grid->center_x()) * kScale); }
  float ny(double y) const { return static_cast<float>((y - grid->center_y()) * kScale); }

  // Whether entity `e` may start at (x, y): free on its medium's ground (a boat's with its
  // clearance from the shore).
  bool free_at(const entity &e, double x, double y) const {
    const WorldSpawns::start_ground g = ground_of(e);
    if (!g.grid || g.grid->occupied(x, y))
      return false;
    int r = 0, c = 0;
    g.grid->sample_of(x, y, r, c);
    return !g.sdf || (*g.sdf)[static_cast<std::size_t>(r) * g.grid->cols + c] >=
                         (floats(e) ? g.clearance : 0.0);
  }
  // A free point of its medium's ground at least `clearance` from anything blocked (a boat's at
  // least its own), kept kSpacing from the placed entities' starts (when it can be), drawn the
  // same way on every platform.
  std::array<double, 2> draw_point(double clearance, const entity *self) {
    std::array<double, 2> best{grid->center_x(), grid->center_y()};
    const WorldSpawns::start_ground g = ground_of(*self);
    const cvc::nav::world_grid &on = g.grid ? *g.grid : *grid;
    const std::vector<float> &dist = g.sdf ? *g.sdf : sdf;
    if (floats(*self))
      clearance = std::max(clearance, g.clearance);
    for (int attempt = 0; attempt < 24; ++attempt) {
      const std::vector<std::array<double, 2>> p =
          cvc::nav::free_points(on, dist, 0x9e3779b97f4a7c15ull ^ (++draws), 1, clearance, 0.0);
      if (p.empty())
        break;
      best = p[0];
      bool crowded = false;
      for (const entity &e : ents)
        if (&e != self && e.agent && std::hypot(e.sx - best[0], e.sy - best[1]) < kSpacing)
          crowded = true;
      if (!crowded)
        break;
    }
    return best;
  }

  // The catalog: each archetype's stats, plant, controllers and look, in the document's order.
  void publish_catalog() {
    if (vkey.empty() || !root())
      return;
    std::string names;
    for (const vehicle_archetype &a : archetypes) {
      names += (names.empty() ? "" : "\n") + a.id;
      const std::string at = vkey + ".archetypes." + a.id + ".";
      const ph::vehicle_desc d = vehicle_desc_of(a);
      const vehicle_stats s = vehicle_stats_of(d);
      put(at + "plant", a.plant);
      put(at + "preset", a.preset);
      put(at + "medium", vehicle_plant_floats(a.plant) ? "water" : "land");
      std::string ctl;
      for (const std::string &c : a.controllers)
        ctl += (ctl.empty() ? "" : "\n") + c;
      put(at + "controllers", ctl);
      put(at + "length", num(s.length));
      put(at + "width", num(s.width));
      put(at + "height", num(s.height));
      put(at + "mass", num(s.mass));
      put(at + "power_kw", num(s.power_kw));
      put(at + "top_speed", num(s.top_speed));
      put(at + "wheelbase", num(s.wheelbase));
      put(at + "track", num(d.track()));
      put(at + "wheel_radius", num(d.axles.empty() ? 0.4 : d.axles.front().wheel_radius));
      put(at + "look.kind", a.look.kind);
      put(at + "look.path", a.look.path);
      put(at + "look.uri", a.look.uri);
      put(at + "look.subpath", a.look.subpath);
      put(at + "look.scale", num(a.look.scale));
      put(at + "look.yaw", num(a.look.yaw));
      put(at + "look.z", num(a.look.z));
      put(at + "look.fit", a.look.fit ? "1" : "0");
      for (const char *r : {"mass", "max_speed"}) {
        const auto it = a.ranges.find(r);
        const double fallback = std::string(r) == "mass" ? d.mass : d.max_speed;
        put(at + "ranges." + r + ".lo", num(it != a.ranges.end() ? it->second.first : fallback));
        put(at + "ranges." + r + ".hi", num(it != a.ranges.end() ? it->second.second : fallback));
      }
    }
    put(vkey + ".archetypes.names", names);
    put(vkey + ".archetypes.count", std::to_string(archetypes.size()));
  }

  // A setting from the spawn keys: an absolute value when given, else a fraction of the
  // archetype's range (<setting>_t, from 0 to 1), else 0 (the archetype's).
  double setting(const vehicle_archetype &a, const std::string &name) const {
    try {
      const std::string abs = get("spawn." + name);
      if (!abs.empty() && std::stod(abs) > 0.0)
        return std::stod(abs);
      const std::string t = get("spawn." + name + "_t");
      const auto it = a.ranges.find(name);
      if (!t.empty() && it != a.ranges.end()) {
        const double f = std::clamp(std::stod(t), 0.0, 1.0);
        return it->second.first + f * (it->second.second - it->second.first);
      }
    } catch (const std::exception &) {
    }
    return 0.0;
  }

  entity_spec spec_from_keys() const {
    entity_spec s;
    s.archetype = get("selected");
    s.name = get("spawn.name");
    try {
      const std::string t = get("spawn.team");
      s.team = t.empty() ? 0 : std::stoi(t);
    } catch (const std::exception &) {
    }
    parse_rgb(get("spawn.color"), s.color);
    parse_rgb(get("spawn.color2"), s.color2);
    s.metadata = parse_metadata(get("spawn.metadata"));
    s.controller = get("spawn.controller");
    s.plant = get("spawn.plant");
    s.policy = get("spawn.policy");
    s.policy_sha256 = get("spawn.policy_sha256");
    if (const vehicle_archetype *a = find(s.archetype)) {
      s.mass = setting(*a, "mass");
      s.max_speed = setting(*a, "max_speed");
    }
    const std::string at = get("spawn.at"); // "x,y" or "x,y,yaw_deg"
    if (!at.empty()) {
      std::stringstream in(at);
      std::string part;
      std::vector<double> v;
      while (std::getline(in, part, ','))
        try {
          v.push_back(std::stod(part));
        } catch (const std::exception &) {
        }
      if (v.size() >= 2) {
        s.placed = true;
        s.x = v[0];
        s.y = v[1];
        if (v.size() >= 3)
          s.yaw = v[2] * kPi / 180.0;
      }
    }
    return s;
  }

  // The selected archetype as a spawn would make it now (the settings applied).
  void publish_selection() {
    if (vkey.empty() || !root())
      return;
    const vehicle_archetype *a = find(get("selected"));
    if (!a) {
      put(vkey + ".selection.summary", "");
      return;
    }
    const entity_spec s = spec_from_keys();
    const ph::vehicle_desc d = vehicle_desc_of(*a, s.mass, s.max_speed);
    const vehicle_stats st = vehicle_stats_of(d);
    const std::string at = vkey + ".selection.";
    put(at + "mass", num(st.mass));
    put(at + "top_speed", num(st.top_speed));
    put(at + "power_kw", num(st.power_kw));
    put(at + "plant", a->plant);
    std::string ctl;
    for (const std::string &c : a->controllers)
      ctl += (ctl.empty() ? "" : "\n") + c;
    put(at + "controllers", ctl);
    const auto range = [&](const char *r, const char *unit) {
      const auto it = a->ranges.find(r);
      return it == a->ranges.end()
                 ? std::string(" (fixed)")
                 : " (" + num(it->second.first) + "-" + num(it->second.second) + " " + unit + ")";
    };
    put(at + "mass_text", fixed(st.mass, 0) + " kg" + range("mass", "kg"));
    put(at + "top_speed_text",
        (st.top_speed > 0.0 ? fixed(st.top_speed, 1) + " m/s" : std::string("unlimited")) +
            range("max_speed", "m/s"));
    put(at + "summary",
        fixed(st.length, 2) + " x " + fixed(st.width, 2) + " x " + fixed(st.height, 2) + " m, " +
            fixed(st.mass, 0) + " kg, " + fixed(st.power_kw, 0) + " kW, top " +
            (st.top_speed > 0.0 ? fixed(st.top_speed, 1) + " m/s" : std::string("unlimited")) +
            ", wheelbase " + fixed(st.wheelbase, 2) + " m, " + a->plant);
  }

  // A summary fleet's vehicles have keys and a stats line of their own only when vehicles.watch
  // names them (a camera's, a panel's: ids joined by ',') or they have an intent: those ids.
  std::set<std::uint64_t> keyed_ids() const {
    std::set<std::uint64_t> keyed;
    bool quiet_fleets = false;
    for (const fleet_group &g : groups)
      quiet_fleets = quiet_fleets || g.summary();
    if (!quiet_fleets)
      return keyed;
    std::stringstream in(get("watch"));
    for (std::string part; std::getline(in, part, ',');) {
      char *end = nullptr;
      const unsigned long long v = std::strtoull(part.c_str(), &end, 10);
      if (end && end != part.c_str())
        keyed.insert(v);
    }
    std::lock_guard<std::mutex> lock(mu);
    for (const auto &[id, it] : intents)
      keyed.insert(id);
    return keyed;
  }
  // A summary fleet's vehicle that nothing names (keyed_ids) and no intent steers: its fleet's
  // count stands for it.
  bool quiet(const entity &e, const std::set<std::uint64_t> &keyed) const {
    return e.group >= 0 && static_cast<std::size_t>(e.group) < groups.size() &&
           groups[static_cast<std::size_t>(e.group)].summary() && !keyed.count(e.id) &&
           e.applied == entity_intent{};
  }
  // The line each summary fleet's quiet vehicles share, after the others' (`quiet` counts them).
  std::string quiet_lines(const std::vector<int> &quiet) const {
    std::string out;
    for (std::size_t gi = 0; gi < groups.size(); ++gi)
      if (quiet[gi])
        out += (out.empty() ? "" : "\n") + std::string("fleet ") + groups[gi].start.fleet + "  " +
               std::to_string(quiet[gi]) + " " + groups[gi].start.archetype + " (keys: summary)";
    return out;
  }

  void publish_entities() {
    publish_groups();
    if (ekey.empty() || !root())
      return;
    const std::set<std::uint64_t> keyed = keyed_ids();
    std::set<std::string> now;
    std::string ids, summary;
    std::vector<int> quiet_count(groups.size(), 0);
    for (const entity &e : ents) {
      if (quiet(e, keyed)) {
        ++quiet_count[static_cast<std::size_t>(e.group)];
        continue;
      }
      const entity_info in = info(e);
      const std::string id = std::to_string(e.id);
      now.insert(id);
      ids += (ids.empty() ? "" : ",") + id;
      const std::string at = ekey + "." + id + ".";
      if (!published.count(id)) { // the identity once; the pose and telemetry every publish
        put(at + "archetype", e.spec.archetype);
        put(at + "name", e.spec.name);
        put(at + "team", std::to_string(e.spec.team));
        put(at + "color", rgb_text(e.spec.color));
        put(at + "color2", rgb_text(e.spec.color2));
        put(at + "controller", e.spec.controller);
        put(at + "plant", e.plant);
        put(at + "policy", e.spec.policy.empty() ? std::string("default") : e.spec.policy);
        put(at + "mass", num(e.desc.mass));
        put(at + "max_speed", num(e.desc.max_speed));
        for (const auto &[k, v] : e.spec.metadata)
          put(at + "metadata." + k, v);
      }
      put(at + "placed", in.placed ? "1" : "0");
      put(at + "x", num(in.x));
      put(at + "y", num(in.y));
      put(at + "z", num(in.z));
      put(at + "yaw", num(in.yaw));
      put(at + "speed", num(in.speed));
      put(at + "power_kw", num(in.power_kw));
      put(at + "energy_wh", num(in.energy_wh));
      put(at + "wh_per_km", num(in.wh_per_km));
      put(at + "distance", num(in.distance));
      put(at + "wheel_load", num(in.wheel_load));
      put(at + "aero_force", num(in.aero_force));
      put(at + "gear", std::to_string(in.gear));
      put(at + "command.steer", num(in.command_steer));
      put(at + "command.accel", num(in.command_accel));
      put(at + "command.by", e.command_by);
      summary += (summary.empty() ? "" : "\n") + id + "  " + e.spec.name + "  " + e.spec.archetype +
                 "  " + e.spec.controller + "  " +
                 (in.placed ? fixed(in.speed, 1) + " m/s" : std::string("waiting"));
    }
    if (const std::string q = quiet_lines(quiet_count); !q.empty())
      summary += (summary.empty() ? "" : "\n") + q;
    cvc::state *r = root();
    for (const std::string &id : published)
      if (!now.count(id))
        if (cvc::state *t = r->findDescendant(ekey))
          t->removeChild(id);
    published = now;
    put(ekey + ".ids", ids);
    put(ekey + ".count", std::to_string(ents.size()));
    put(ekey + ".summary", summary);
    put(ekey + ".rev", std::to_string(rev));
  }
};

WorldVehicles::WorldVehicles(cvc::app *app, ph::world &world,
                             std::vector<vehicle_archetype> archetypes, std::string vehicles_key,
                             std::string entities_key, spawn_rules spawns, std::string spawns_key)
    : _impl(std::make_unique<impl>()) {
  impl &d = *_impl;
  d.app = app;
  d.world = &world;
  d.spawns = std::make_unique<WorldSpawns>(app, std::move(spawns), std::move(spawns_key));
  d.archetypes = std::move(archetypes);
  resolve_vehicle_looks(d.archetypes, "", {}); // a look a host left as named: found here
  d.vkey = std::move(vehicles_key);
  d.ekey = std::move(entities_key);
  d.status = d.archetypes.empty() ? "no vehicles: the world document's world: vehicles: is empty"
                                  : "waiting for the world's grid";
  if (d.root() && !d.vkey.empty()) {
    d.publish_catalog();
    // The panel's keys, seeded where a host or init: has not set them.
    d.seed(d.vkey + ".selected", d.archetypes.empty() ? "" : d.archetypes.front().id);
    d.seed(d.vkey + ".spawn.team", "0");
    d.seed(d.vkey + ".spawn.color", "0.85,0.7,0.2");
    d.seed(d.vkey + ".spawn.color2", "0.12,0.12,0.14");
    d.seed(d.vkey + ".spawn.controller", "nav@1");
    d.seed(d.vkey + ".avoid", d.avoid ? "1" : "0");
    d.seed(d.vkey + ".sense", d.sense);
    d.seed(d.vkey + ".spawn.mass_t", "0.5");
    d.seed(d.vkey + ".spawn.max_speed_t", "1");
    for (const char *req : {"spawn.request", "remove.request", "clear.request"})
      d.seed(d.vkey + "." + req, "0");
    d.seen_spawn = d.get("spawn.request");
    d.seen_remove = d.get("remove.request");
    d.seen_clear = d.get("clear.request");
    d.put(d.vkey + ".status", d.status);
    d.publish_selection();
  }
  d.publish_entities();
}

WorldVehicles::~WorldVehicles() {
  // The vehicles leave the world with the runtime (the world may outlive it).
  try {
    _impl->unplace_all(false);
  } catch (const std::exception &) {
  }
}

const std::vector<vehicle_archetype> &WorldVehicles::archetypes() const {
  return _impl->archetypes;
}

void WorldVehicles::set_ground(std::shared_ptr<const nav::world_grid> grid,
                               std::vector<ph::body_id> ground_bodies,
                               const std::string &world_name,
                               std::shared_ptr<const cvc::world::world_geometry> world) {
  impl &d = *_impl;
  d.unplace_all(false);
  d.drop_still_water(); // the boats' still water was the old grid's
  d.terrain = world ? world->terrain() : nullptr;
  d.spawns->set_world(world_name, grid, std::move(world));
  d.grid = std::move(grid);
  d.ground = std::move(ground_bodies);
  d.status = d.grid ? "ready" : "waiting for the world's grid";
  for (impl::fleet_group &g : d.groups) // a new grid: each fleet is laid out on it again
    g.laid_out = false;
  try {
    d.build_navigator();
    if (d.ready)
      d.update_needs();
  } catch (const std::exception &ex) {
    d.fleets.clear();
    d.ready = false;
    d.status = std::string("the navigator could not be made: ") + ex.what();
  }
  d.place_all();
  d.put(d.vkey + ".status", d.status);
}

void WorldVehicles::world_reset(std::vector<ph::body_id> ground_bodies) {
  impl &d = *_impl;
  d.unplace_all(true);
  d.still_water = ph::field_id{}; // it went with the old world
  d.ground = std::move(ground_bodies);
  try {
    d.build_navigator();
    if (d.ready)
      d.update_needs();
  } catch (const std::exception &ex) {
    d.fleets.clear();
    d.ready = false;
    d.status = std::string("the navigator could not be made: ") + ex.what();
  }
  d.place_all();
}

std::uint64_t WorldVehicles::spawn(const entity_spec &spec, std::string *why) {
  impl &d = *_impl;
  const auto refuse = [&](const std::string &w) -> std::uint64_t {
    if (why)
      *why = w;
    return 0;
  };
  const vehicle_archetype *a = d.find(spec.archetype);
  if (!a)
    return refuse("no vehicle archetype '" + spec.archetype + "'");
  impl::entity e;
  e.id = d.next_id;
  e.spec = spec;
  e.plant = spec.plant.empty() ? a->plant : spec.plant;
  if (!vehicle_plant_known(e.plant))
    return refuse(spec.plant.empty()
                      ? "vehicle '" + a->id + "': plant '" + a->plant + "' is not one"
                      : "no plant '" + e.plant + "' (jolt, kinematic, dynamic, dynamic4w)");
  if (e.spec.controller.empty())
    e.spec.controller = a->controllers.empty() ? "none" : a->controllers.front();
  if (std::find(a->controllers.begin(), a->controllers.end(), e.spec.controller) ==
      a->controllers.end())
    return refuse("vehicle '" + a->id + "' does not allow controller '" + e.spec.controller + "'");
  if (e.spec.name.empty())
    e.spec.name = a->id + " " + std::to_string(e.id);
  try {
    e.desc = vehicle_desc_of(*a, spec.mass, spec.max_speed);
  } catch (const std::exception &ex) {
    return refuse(ex.what());
  }
  e.posed = e.spec.controller == "pose";
  if (e.posed)
    e.command_by = "pose";
  d.ents.push_back(std::move(e));
  impl::entity &placed = d.ents.back();
  if (placed.posed && spec.placed) { // its first pose: where the spawn puts its rear axle
    entity_pose first;
    first.x = spec.x;
    first.y = spec.y;
    first.heading = std::isfinite(spec.yaw) ? spec.yaw : 0.0;
    first.rear_axle = true;
    d.take_pose(placed, first, d.world->tick());
  }
  if ((d.ready && !d.hold_placing) || placed.posed) {
    if (std::string w = d.place(placed); !w.empty()) {
      d.ents.pop_back();
      {
        std::lock_guard<std::mutex> lock(d.mu);
        d.taken.erase(d.next_id);
      }
      return refuse(w);
    }
  }
  ++d.next_id;
  ++d.rev;
  if (placed.posed) {
    ++d.posed_count;
    // The packed values are read again at the next step, for its record in them.
    d.seen_own_packed.assign(1, '\0');
    d.seen_table_packed.assign(1, '\0');
  }
  {
    std::lock_guard<std::mutex> lock(d.mu);
    d.live.insert(placed.id);
    if (placed.posed)
      d.posed_ids.insert(placed.id);
  }
  return placed.id;
}

std::size_t WorldVehicles::spawn_starts(const std::vector<vehicle_start> &starts,
                                        std::vector<std::string> *warnings) {
  impl &d = *_impl;
  d.starts.insert(d.starts.end(), starts.begin(), starts.end());
  std::size_t made = 0;
  for (std::size_t i = 0; i < starts.size(); ++i) {
    const vehicle_start &st = starts[i];
    if (!st.fleet.empty()) {
      // A fleet: its vehicles spawned unplaced, then laid out and placed together.
      if (d.group_named(st.fleet)) {
        if (warnings)
          warnings->push_back("ari: world: vehicles: start: [" + std::to_string(i) + "]: fleet '" +
                              st.fleet + "' is there already");
        continue;
      }
      // A fleet lays out and drives its vehicles; a `pose` entity is placed by its poses alone.
      const vehicle_archetype *fa = d.find(st.archetype);
      if (st.controller == "pose" || (st.controller.empty() && fa && !fa->controllers.empty() &&
                                      fa->controllers.front() == "pose")) {
        if (warnings)
          warnings->push_back("ari: world: vehicles: start: [" + std::to_string(i) + "]: fleet '" +
                              st.fleet + "': its vehicles are driven, not posed (controller pose)");
        continue;
      }
      d.groups.emplace_back();
      const std::size_t gi = d.groups.size() - 1;
      d.groups[gi].start = st;
      d.groups[gi].seed = st.seed;
      d.groups[gi].seen_restart = d.get("fleets." + st.fleet + ".restart.request");
      d.groups[gi].seen_reseed = d.get("fleets." + st.fleet + ".reseed.request");
      d.groups[gi].status = "waiting for the world's grid";
      // Its navigator's keys, as the start asks (writing one switches it live, poll_groups).
      impl::fleet_group &g = d.groups[gi];
      g.sense = st.sense;
      g.belief = st.belief.empty() ? "shared" : st.belief;
      g.policy = !st.policy_data.empty() ? "data:" + st.policy_data
                 : st.policy.empty()     ? "trained"
                                         : st.policy;
      g.metric = st.metric.empty() ? "policy" : st.metric;
      g.base_plant = fa ? fa->plant : std::string();
      g.plants = st.plants;
      if (g.plants.empty() && !st.plant.empty())
        g.plants.cycle.push_back(st.plant);
      const std::string at = d.vkey + ".fleets." + st.fleet + ".";
      if (!d.vkey.empty()) {
        d.put(at + "sense", d.sense_of(static_cast<int>(gi)));
        d.put(at + "belief", g.belief);
        d.put(at + "policy", g.policy);
        d.put(at + "metric", g.metric);
        d.put(at + "plant", g.plants.empty() ? g.base_plant : g.plants.text());
      }
      g.seen_sense = d.get("fleets." + st.fleet + ".sense");
      g.seen_belief = d.get("fleets." + st.fleet + ".belief");
      g.seen_policy = d.get("fleets." + st.fleet + ".policy");
      g.seen_metric = d.get("fleets." + st.fleet + ".metric");
      g.seen_plant = d.get("fleets." + st.fleet + ".plant");
      d.hold_placing = true;
      int got = 0;
      std::string why;
      for (int k = 0; k < st.count; ++k) {
        entity_spec s;
        s.archetype = st.archetype;
        s.name = (st.name.empty() ? st.fleet : st.name) + " " + std::to_string(k + 1);
        s.team = st.team;
        s.controller = st.controller;
        s.plant = g.plants.of(k, -1); // its turn in the list; its team's once it has one (place)
        if (st.has_heading)
          s.yaw = st.heading * kPi / 180.0;
        if (!spawn(s, &why))
          break;
        d.ents.back().group = static_cast<int>(gi);
        d.ents.back().member = k;
        ++got;
      }
      d.hold_placing = false;
      d.resolve_policy(gi); // before they are placed: they start on it
      if (got < st.count && warnings)
        warnings->push_back("ari: world: vehicles: start: [" + std::to_string(i) + "]: fleet '" +
                            st.fleet + "': " + why);
      if (d.ready)
        for (impl::entity &e : d.ents)
          if (e.group == static_cast<int>(gi) && !e.agent)
            if (std::string w = d.place(e); !w.empty())
              d.status = "entity " + std::to_string(e.id) + ": " + w;
      ++d.rev;
      made += static_cast<std::size_t>(got);
      continue;
    }
    for (int k = 0; k < st.count; ++k) {
      entity_spec s;
      s.archetype = st.archetype;
      s.name = st.name.empty() || st.count == 1 ? st.name : st.name + " " + std::to_string(k + 1);
      s.team = st.team;
      s.controller = st.controller;
      s.plant = st.plant;
      s.policy = st.policy;
      s.placed = st.placed;
      s.x = st.x;
      s.y = st.y;
      if (st.has_heading)
        s.yaw = st.heading * kPi / 180.0;
      std::string why;
      if (!spawn(s, &why)) {
        if (warnings)
          warnings->push_back("ari: world: vehicles: start: [" + std::to_string(i) + "]: " + why);
        break;
      }
      ++made;
    }
  }
  return made;
}

bool WorldVehicles::despawn(std::uint64_t id) {
  impl &d = *_impl;
  for (auto it = d.ents.begin(); it != d.ents.end(); ++it)
    if (it->id == id) {
      d.remove_from_fleet(*it);
      if (it->posed)
        --d.posed_count;
      d.ents.erase(it);
      ++d.rev;
      std::lock_guard<std::mutex> lock(d.mu);
      d.live.erase(id);
      d.intents.erase(id);
      d.posed_ids.erase(id);
      d.pending.erase(id);
      d.taken.erase(id);
      return true;
    }
  return false;
}

void WorldVehicles::clear() {
  impl &d = *_impl;
  d.unplace_all(false);
  d.ents.clear();
  for (std::size_t gi = 0; gi < d.groups.size(); ++gi) // the fleets' own navigators go with them
    d.drop_navigators(gi);
  d.groups.clear();
  d.starts.clear();
  d.posed_count = 0;
  ++d.rev;
  std::lock_guard<std::mutex> lock(d.mu);
  d.live.clear();
  d.intents.clear();
  d.posed_ids.clear();
  d.pending.clear();
  d.taken.clear();
}

bool WorldVehicles::set_intent(std::uint64_t id, const entity_intent &intent) {
  impl &d = *_impl;
  if (!finite_intent(intent))
    return false;
  std::lock_guard<std::mutex> lock(d.mu);
  if (!d.live.count(id))
    return false;
  d.intents[id] = intent;
  return true;
}

bool WorldVehicles::clear_intent(std::uint64_t id) { return set_intent(id, entity_intent{}); }

entity_intent WorldVehicles::intent(std::uint64_t id) const {
  const impl &d = *_impl;
  std::lock_guard<std::mutex> lock(d.mu);
  const auto it = d.intents.find(id);
  return it == d.intents.end() ? entity_intent{} : it->second;
}

void WorldVehicles::set_intents_key(std::string key) { _impl->ikey = std::move(key); }

bool WorldVehicles::set_pose(std::uint64_t id, const entity_pose &pose) {
  const entity_pose_update one{id, pose};
  return set_poses(std::span<const entity_pose_update>(&one, 1)) == 1;
}

std::size_t WorldVehicles::set_poses(std::span<const entity_pose_update> poses) {
  impl &d = *_impl;
  std::size_t took = 0;
  std::lock_guard<std::mutex> lock(d.mu);
  for (const entity_pose_update &u : poses)
    if (finite_pose(u.pose) && d.posed_ids.count(u.id)) {
      d.pending[u.id] = u.pose;
      ++took;
    }
  return took;
}

bool WorldVehicles::pose(std::uint64_t id, entity_pose &out) const {
  const impl &d = *_impl;
  std::lock_guard<std::mutex> lock(d.mu);
  if (const auto it = d.pending.find(id); it != d.pending.end()) {
    out = it->second;
    return true;
  }
  if (const auto it = d.taken.find(id); it != d.taken.end()) {
    out = it->second;
    return true;
  }
  return false;
}

void WorldVehicles::set_poses_key(std::string key) { _impl->pkey = std::move(key); }
const std::string &WorldVehicles::poses_key() const { return _impl->pkey; }

void WorldVehicles::set_pose_jump(double metres) {
  if (!(metres > 0.0) || !std::isfinite(metres))
    throw std::invalid_argument(
        "WorldVehicles::set_pose_jump: the distance must be finite and > 0");
  _impl->pose_jump = metres;
}
double WorldVehicles::pose_jump() const { return _impl->pose_jump; }
void WorldVehicles::set_wind(std::shared_ptr<const physics::flow_velocity> wind) {
  _impl->wind = std::move(wind);
}
const std::string &WorldVehicles::intents_key() const { return _impl->ikey; }

WorldSpawns &WorldVehicles::spawns() { return *_impl->spawns; }
const WorldSpawns &WorldVehicles::spawns() const { return *_impl->spawns; }

void WorldVehicles::poll() {
  impl &d = *_impl;
  d.spawns->poll();
  d.sync_pose_table(*this);
  if (!d.root() || d.vkey.empty())
    return;
  const std::string sel = d.get("selected") + "|" + d.get("spawn.mass") + "|" +
                          d.get("spawn.mass_t") + "|" + d.get("spawn.max_speed") + "|" +
                          d.get("spawn.max_speed_t");
  if (sel != d.seen_sel) {
    d.seen_sel = sel;
    d.publish_selection();
  }
  if (const std::string m = d.get("sense"); (m == "known" || m == "fog") && m != d.sense)
    set_sense(m);
  d.poll_groups();
  if (const std::string a = d.get("avoid"); a == "0" || a == "1")
    if ((a == "1") != d.avoid)
      set_avoid(a == "1");
  if (const std::string r = d.get("spawn.request"); r != d.seen_spawn) {
    d.seen_spawn = r;
    // spawn.count of them (1 when empty or not a number), each at a free point of its own when
    // spawn.at names none; the first refusal stops the rest.
    int count = 1;
    try {
      if (const std::string c = d.get("spawn.count"); !c.empty())
        count = std::clamp(std::stoi(c), 1, kMaxSpawnCount);
    } catch (const std::exception &) {
    }
    std::string why, status;
    std::uint64_t id = 0;
    int made = 0;
    for (int i = 0; i < count; ++i) {
      const entity_spec s = d.spec_from_keys();
      const std::uint64_t got = spawn(s, &why);
      if (!got)
        break;
      id = got;
      ++made;
    }
    if (made == 0)
      status = why;
    else if (made == 1)
      status = "spawned " + std::to_string(id) + ": " + d.ents.back().spec.name;
    else
      status = "spawned " + std::to_string(made) + ", the last " + std::to_string(id) + ": " +
               d.ents.back().spec.name + (made < count ? " (" + why + ")" : std::string());
    d.put(d.vkey + ".spawn.status", status);
    d.put(d.vkey + ".spawn.last", std::to_string(id));
  }
  if (const std::string r = d.get("remove.request"); r != d.seen_remove) {
    d.seen_remove = r;
    try {
      const std::string id = d.get("remove.id");
      if (!id.empty())
        despawn(std::stoull(id));
    } catch (const std::exception &) {
    }
  }
  if (const std::string r = d.get("clear.request"); r != d.seen_clear) {
    d.seen_clear = r;
    clear();
  }
  d.put(d.vkey + ".status", d.status);
  d.put(d.vkey + ".count", std::to_string(d.ents.size()));
}

void WorldVehicles::set_avoid(bool avoid) {
  impl &d = *_impl;
  if (avoid == d.avoid)
    return;
  d.avoid = avoid;
  if (!d.vkey.empty())
    d.put(d.vkey + ".avoid", avoid ? "1" : "0");
  // The separation is each navigator's: changed in place, from its next tick.
  d.cfg.sep_radius = avoid ? kAvoidRadius * d.cfg.veh.rr : 0.0f;
  d.cfg.sep_gain = avoid ? kAvoidGain * d.cfg.veh.rr : 0.0f;
  d.cfg.sep_side = avoid ? kAvoidSide : 0.0f;
  d.cfg.sep_side_radius = avoid ? kAvoidSideRadius * d.cfg.veh.rr : 0.0f;
  for (auto &[name, f] : d.fleets)
    if (f->sim) { // sized by its own vehicle (a fleet on the trained metric has the trainer's)
      f->sim->set_separation(avoid ? kAvoidRadius * f->rr : 0.0f,
                             avoid ? kAvoidGain * f->rr : 0.0f);
      f->sim->set_separation_side(d.cfg.sep_side, avoid ? kAvoidSideRadius * f->rr : 0.0f);
    }
}

bool WorldVehicles::avoid() const { return _impl->avoid; }

void WorldVehicles::set_movers(std::vector<std::array<double, 2>> xy) {
  _impl->movers = std::move(xy);
}

const std::vector<std::array<double, 2>> &WorldVehicles::movers() const { return _impl->movers; }

bool WorldVehicles::set_sense(const std::string &mode) {
  impl &d = *_impl;
  if (mode != "known" && mode != "fog")
    return false;
  if (mode == d.sense)
    return true;
  d.sense = mode;
  if (!d.vkey.empty())
    d.put(d.vkey + ".sense", mode);
  for (impl::fleet_group &g : d.groups)
    if (g.sense.empty() && !d.vkey.empty()) {
      d.put(d.vkey + ".fleets." + g.start.fleet + ".sense", mode);
      g.seen_sense = mode;
    }
  // What a navigator knows is its config and its prior: they are made again, on the grid they
  // have, and every vehicle goes back to its start.
  if (d.grid) {
    d.unplace_all(false);
    try {
      d.build_navigator();
      if (d.ready)
        d.update_needs();
    } catch (const std::exception &ex) {
      d.fleets.clear();
      d.ready = false;
      d.status = std::string("the navigator could not be made: ") + ex.what();
    }
    d.place_all();
  }
  return true;
}

const std::string &WorldVehicles::sense() const { return _impl->sense; }

bool WorldVehicles::belief(const std::string &plant, std::vector<std::uint8_t> &out, int &rows,
                           int &cols) const {
  const impl &d = *_impl;
  const auto it = d.fleets.find(plant);
  if (it == d.fleets.end() || !it->second || !it->second->sim || !d.grid)
    return false;
  rows = it->second->sim->rows();
  cols = it->second->sim->cols();
  const std::uint8_t *occ = it->second->sim->belief_occ(0);
  if (!occ || rows <= 0 || cols <= 0)
    return false;
  out.assign(occ, occ + static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols));
  return true;
}

void WorldVehicles::before_step() {
  impl &d = *_impl;
  d.stepping = d.posing = false;
  if (d.ents.empty())
    return;
  if (d.posed_count > 0) { // the `pose` entities, grid or none
    d.pose_step();
    d.posing = true;
  }
  if (!d.ready)
    return;
  if (d.steps++ % static_cast<std::uint64_t>(d.every) == 0) {
    d.navigator_tick();
    if (d.have_clock) {
      const std::uint64_t phase = d.clock_tick % static_cast<std::uint64_t>(d.every);
      if (!d.have_phase || phase != d.phase) {
        d.phase = phase;
        d.have_phase = true;
        ++d.rev; // the room's entities carry it (kinematics)
      }
    }
  }
  d.before_world_step();
  d.stepping = true;
}

void WorldVehicles::before_step(std::uint64_t clock_tick) {
  _impl->have_clock = true;
  _impl->clock_tick = clock_tick;
  before_step();
}

void WorldVehicles::after_step() {
  impl &d = *_impl;
  if (d.stepping)
    d.after_world_step();
  else if (d.posing)
    d.count_contacts();
  d.stepping = d.posing = false;
}

void WorldVehicles::publish() { _impl->publish_entities(); }

void WorldVehicles::reseed(std::uint64_t seed) {
  impl &d = *_impl;
  // splitmix64 of the seed: nearby seeds draw unrelated points.
  std::uint64_t z = seed + 0x9e3779b97f4a7c15ull;
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
  d.draws = z ^ (z >> 31);
  d.spawns->reseed(seed);
  if (d.ents.empty())
    d.next_id = 1;
}

void WorldVehicles::set_wander(bool wander) { _impl->wander = wander; }

void WorldVehicles::set_drive_telemetry(bool on) {
  impl &d = *_impl;
  d.drive_tel = on;
  for (auto &kv : d.fleets) {
    kv.second->sim->set_capture_drive_telemetry(on);
    kv.second->tel_fresh = false;
  }
}
bool WorldVehicles::drive_telemetry() const { return _impl->drive_tel; }

bool WorldVehicles::command(std::uint64_t id, double steer, double accel) {
  for (impl::entity &e : _impl->ents)
    if (e.id == id) {
      if (_impl->index(e) < 0)
        return false;
      e.cmd_steer = steer;
      e.cmd_accel = accel;
      return true;
    }
  return false;
}

bool WorldVehicles::set_driver(std::uint64_t id, const std::string &pid) {
  impl &d = *_impl;
  for (impl::entity &e : d.ents)
    if (e.id == id) {
      if (e.spec.driver != pid) {
        e.spec.driver = pid;
        ++d.rev;
      }
      return true;
    }
  return false;
}

std::string WorldVehicles::driver(std::uint64_t id) const {
  for (const impl::entity &e : _impl->ents)
    if (e.id == id)
      return e.spec.driver;
  return std::string();
}

std::vector<std::string> WorldVehicles::fleets() const {
  std::vector<std::string> out;
  for (const impl::fleet_group &g : _impl->groups)
    out.push_back(g.start.fleet);
  return out;
}

std::shared_ptr<const fleet_poses> WorldVehicles::poses(const std::string &fleet) const {
  const std::shared_ptr<const impl::pose_book> book = std::atomic_load(&_impl->book);
  if (!book)
    return nullptr;
  const auto it = book->find(fleet);
  return it == book->end() ? nullptr : it->second;
}

const std::vector<vehicle_start> &WorldVehicles::starts() const { return _impl->starts; }

bool WorldVehicles::rally(const std::string &fleet, int team, double &x, double &y) const {
  for (const impl::fleet_group &g : _impl->groups)
    if (g.start.fleet == fleet && team >= 0 && static_cast<std::size_t>(team) < g.rally.size()) {
      x = g.rally[static_cast<std::size_t>(team)][0];
      y = g.rally[static_cast<std::size_t>(team)][1];
      return true;
    }
  return false;
}

bool WorldVehicles::move_rally(const std::string &fleet, int team, double x, double y) {
  impl &d = *_impl;
  for (std::size_t gi = 0; gi < d.groups.size(); ++gi)
    if (d.groups[gi].start.fleet == fleet)
      return d.move_rally(gi, team, x, y);
  return false;
}

bool WorldVehicles::restart_fleet(const std::string &fleet, bool reseed) {
  impl &d = *_impl;
  for (std::size_t gi = 0; gi < d.groups.size(); ++gi)
    if (d.groups[gi].start.fleet == fleet) {
      d.restart_group(gi, reseed);
      return true;
    }
  return false;
}

bool WorldVehicles::set_goal(std::uint64_t id, double x, double y) {
  impl &d = *_impl;
  for (impl::entity &e : d.ents)
    if (e.id == id) {
      impl::fleet *f = d.fleet_of(e);
      const int i = d.index(e);
      if (!f || i < 0)
        return false;
      e.gx = x;
      e.gy = y;
      f->sim->retarget(i, d.nx(x), d.ny(y));
      return true;
    }
  return false;
}

double WorldVehicles::tick_seconds() const {
  return _impl->ready ? _impl->every * _impl->world->config().fixed_dt : 0.0;
}

int WorldVehicles::steps_per_tick() const { return _impl->ready ? _impl->every : 0; }

bool WorldVehicles::kinematics(std::uint64_t id, kinematic_drive &out) const {
  const impl &d = *_impl;
  if (!d.ready || !d.have_phase)
    return false;
  for (const impl::entity &e : d.ents) {
    if (e.id != id)
      continue;
    const impl::fleet *f = d.fleet_of(e);
    if (!f || f->plant != "kinematic" || d.index(e) < 0)
      return false;
    // As kinematics() and the navigator's config give them to nav (in metres here).
    kinematic_drive k;
    k.wheelbase = e.desc.wheelbase();
    k.lock = d.cfg.veh.delta_max;
    k.vmax = e.desc.max_speed > 0.0 ? e.desc.max_speed : 30.0;
    k.a_max = 0.8 * e.desc.max_accel;
    k.allow_reverse = d.cfg.veh.allow_reverse;
    k.rear_x = impl::rear_axle_x(e.desc);
    k.tick_steps = d.every;
    k.nsub = std::max(1, d.cfg.veh.nsub);
    k.phase = d.phase;
    k.max_steer = e.desc.max_steer;
    k.max_accel = e.desc.max_accel;
    k.max_decel = e.desc.max_decel;
    out = k;
    return true;
  }
  return false;
}

bool WorldVehicles::dynamics(std::uint64_t id, dynamic_drive &out) const {
  const impl &d = *_impl;
  if (!d.ready || !d.have_phase)
    return false;
  for (const impl::entity &e : d.ents) {
    if (e.id != id)
      continue;
    const impl::fleet *f = d.fleet_of(e);
    const int i = d.index(e);
    cvc::nav::sim_world::plant_plan plan;
    if (!f || i < 0 ||
        !((f->plant == "dynamic" && f->bike) || (f->plant == "dynamic4w" && f->four)))
      return false;
    if (!f->sim->plant_plan_of(i, plan))
      return false;
    const vehicle_archetype *a = d.find(e.spec.archetype);
    if (!a)
      return false;
    dynamic_drive k;
    k.plant = f->plant;
    k.preset = a->preset;
    k.mass = e.desc.mass;
    k.max_speed = e.desc.max_speed;
    k.max_accel = e.desc.max_accel;
    k.max_decel = e.desc.max_decel;
    k.max_steer = e.desc.max_steer;
    // As sim_world holds an override's command on a plant (in metres here).
    k.lock = d.cfg.veh.delta_max;
    if (plan.max_steer > 0.0f)
      k.lock = std::min(k.lock, static_cast<double>(plan.max_steer));
    k.vmax = plan.vmax / kScale;
    k.a_hold = (plan.a_brake > 0.0f ? plan.a_brake : plan.a_max) / kScale;
    k.tick_steps = d.every;
    k.phase = d.phase;
    if (f->four)
      k.ground = f->four->ground_reference();
    out = k;
    return true;
  }
  return false;
}

double WorldVehicles::ground_height(double x, double y) const { return _impl->ground_height(x, y); }

double vehicle_ground_height(const nav::world_grid *grid, const cvc::height_field *terrain,
                             double x, double y) {
  return ground_at(grid, terrain, x, y);
}

void WorldVehicles::set_tick(double seconds) {
  if (!(seconds > 0.0) || !std::isfinite(seconds))
    throw std::invalid_argument("WorldVehicles::set_tick: the tick must be finite and > 0");
  _impl->tick = seconds;
}
double WorldVehicles::tick() const {
  if (_impl->tick > 0.0)
    return _impl->tick;
  return _impl->world ? cvc::sim_tick(_impl->world->config().fixed_dt) : cvc::kSimTick;
}

std::shared_ptr<const nav::world_grid> WorldVehicles::grid() const { return _impl->grid; }

namespace {
// v rotated by q.
ph::vec3 rotated(const ph::quat &q, const ph::vec3 &v) {
  const ph::vec3 u{q.x, q.y, q.z};
  const ph::vec3 t{2.0 * (u.y * v.z - u.z * v.y), 2.0 * (u.z * v.x - u.x * v.z),
                   2.0 * (u.x * v.y - u.y * v.x)};
  return v + q.w * t +
         ph::vec3{u.y * t.z - u.z * t.y, u.z * t.x - u.x * t.z, u.x * t.y - u.y * t.x};
}
} // namespace

std::vector<entity_plant> WorldVehicles::plants() const {
  const impl &d = *_impl;
  std::vector<entity_plant> out;
  for (const impl::entity &e : d.ents) {
    entity_plant p;
    p.id = e.id;
    p.plant = e.plant;
    p.desc = e.desc;
    const double rear_x = impl::rear_axle_x(e.desc);
    p.rear_x = rear_x;
    const impl::fleet *g = d.fleet_of(e);
    const int i = d.index(e);
    p.placed = g && i >= 0;
    if (e.posed) { // placed from outside: no model drives it
      p.placed = e.box.valid();
      p.model = "pose";
    }
    if (!p.placed || e.posed) {
      out.push_back(std::move(p));
      continue;
    }
    if (g->four) {
      const ph::dyn_4w_plant &pl = *g->four;
      p.model = "dyn4";
      p.exact = true;
      p.dyn4 = pl.params(i);
      p.dyn4_state = pl.body(i);
      p.limits = pl.step_limits(i);
      p.substeps = pl.substeps();
      p.ground_reference = pl.ground_reference();
      p.mu = pl.options().mu;
      p.steer_command = pl.steer_command(i);
      p.accel_command = pl.accel_command(i);
      p.applied_accel = pl.applied_accel(i);
      p.collisions_tick = pl.collisions_tick(i);
    } else if (g->bike) {
      const ph::dyn_bike_plant &pl = *g->bike;
      p.model = "dyn_bike";
      p.exact = true;
      p.bike = pl.params(i);
      p.bike_state = pl.body(i);
      p.limits = pl.step_limits(i);
      p.substeps = pl.substeps();
      p.mu = pl.options().mu;
      p.steer_command = pl.steer_command(i);
      p.accel_command = pl.accel_command(i);
      p.applied_accel = pl.applied_accel(i);
      p.applied_grade = pl.applied_grade(i);
      p.collisions_tick = pl.collisions_tick(i);
    } else if (g->jolt) {
      // The four-wheel model with the vehicle's own numbers, as dyn_4w_plant builds it, its pose
      // and twist read back from the chassis as dyn_4w_plant reads a collision back.
      p.model = "dyn4";
      cvc::world_units::normalized_frame si;
      si.scale = 1.0;
      p.dyn4 = nav::dyn4_params_of(ph::nav_plant::dyn4_si_of(e.desc), si);
      p.substeps = d.every;
      p.limits.L = p.dyn4.lf + p.dyn4.lr;
      p.limits.delta_max = static_cast<float>(e.desc.max_steer);
      p.limits.hdt = static_cast<float>(d.cfg.veh.dt / std::max(1, d.every));
      p.limits.vmax = static_cast<float>(e.desc.max_speed > 0.0 ? e.desc.max_speed : 1e3);
      p.limits.allow_reverse = 1;
      const ph::vehicle_state vs = d.world->vehicle_state(g->jolt->vehicle(i));
      const ph::quat &q = vs.body.rotation;
      nav::diff::dyn4_state<double> &o = p.dyn4_state;
      o.psi = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      const double c = std::cos(o.psi), sn = std::sin(o.psi);
      const double lr = p.dyn4.lr, off = -rear_x - lr;
      const ph::vec3 cg =
          vs.body.position + rotated(q, {rear_x + lr, 0.0, e.desc.center_of_mass.z});
      o.x = cg.x - lr * c;
      o.y = cg.y - lr * sn;
      const ph::vec3 &v = vs.body.linear_velocity, &w = vs.body.angular_velocity;
      o.r = w.z;
      const double vgx = v.x + o.r * off * sn, vgy = v.y - o.r * off * c;
      o.vx = c * vgx + sn * vgy;
      o.vy = -sn * vgx + c * vgy;
      o.zd = v.z;
      o.phi = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
      o.th = std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
      o.phid = c * w.x + sn * w.y;
      o.thd = -sn * w.x + c * w.y;
      o.delta = vs.steer;
      p.steer_command = vs.steer_command;
    } else {
      // Nav's bicycle: the rear axle, the heading and the speed, in world metres. A boat's is the
      // bicycle nav plans it with; dyn_boat_plant drives it, so the bicycle is not `exact` there.
      p.model = "bike";
      p.exact = !g->boat;
      const int n = g->sim->size();
      std::vector<float> pos(2 * static_cast<std::size_t>(n)), head(n), speed(n);
      g->sim->snapshot(pos.data(), head.data(), speed.data(), nullptr, nullptr);
      p.bike_state.x = pos[2 * static_cast<std::size_t>(i)];
      p.bike_state.y = pos[2 * static_cast<std::size_t>(i) + 1];
      p.bike_state.psi = head[i];
      p.bike_state.vx = speed[i];
      const nav::veh_params &vp = g->sim->current_config().veh; // its fleet's (its metric's)
      p.limits.rr = static_cast<float>(vp.rr / kScale);
      p.limits.d_hat = static_cast<float>(vp.d_hat / kScale);
      p.limits.vmax = static_cast<float>(vp.vmax / kScale);
      p.limits.L = static_cast<float>(vp.L / kScale);
      p.limits.delta_max = vp.delta_max;
      p.limits.a_max = static_cast<float>(vp.a_max / kScale);
      p.limits.a_lat_max = static_cast<float>(vp.a_lat_max / kScale);
      p.limits.k_steer = vp.k_steer;
      p.limits.hdt = static_cast<float>(vp.dt / std::max(1, vp.nsub));
      p.limits.allow_reverse = vp.allow_reverse ? 1 : 0;
      p.substeps = std::max(1, vp.nsub);
    }
    // The drive that gave the last command, when its navigator captured it.
    if (const nav::drive_telemetry *t = g->tel_fresh ? g->sim->drive_telemetry_data() : nullptr) {
      std::vector<float> carrots(2 * static_cast<std::size_t>(g->sim->size()));
      g->sim->carrots_world(carrots.data());
      const nav::drive_telemetry &ti = t[i];
      p.drive.known = true;
      p.drive.alpha = ti.alpha;
      p.drive.beta = ti.beta;
      p.drive.gamma = ti.gamma;
      p.drive.goal_x = carrots[2 * static_cast<std::size_t>(i)];
      p.drive.goal_y = carrots[2 * static_cast<std::size_t>(i) + 1];
      p.drive.force_x = ti.ext_fx / kScale;
      p.drive.force_y = ti.ext_fy / kScale;
    }
    // The limits it planned with on the plant (sim_world's plant arm), normalized there.
    nav::sim_world::plant_plan pp;
    if (g->sim->plant_plan_of(i, pp)) {
      static_assert(nav::sim_world::kPlantDiscs == std::tuple_size<decltype(p.plan.body_offsets)>{},
                    "entity_plan's footprint discs are sim_world's");
      entity_plan &pl = p.plan;
      pl.known = true;
      pl.a_max = pp.a_max / kScale;
      pl.vmax = pp.vmax / kScale;
      pl.a_lat = pp.a_lat / kScale;
      pl.wheelbase = pp.wheelbase / kScale;
      pl.max_steer = pp.max_steer;
      pl.steer_rate = pp.steer_rate;
      pl.steer_now = pp.steer_now;
      pl.latency = pp.latency;
      pl.a_brake = pp.a_brake / kScale;
      pl.footprint = pp.footprint;
      pl.body_rr = pp.body_rr / kScale;
      for (std::size_t b = 0; b < pl.body_offsets.size(); ++b)
        pl.body_offsets[b] = pp.body_offsets[b] / kScale;
      pl.parked = pp.parked;
      pl.modulated = pp.modulated;
    }
    out.push_back(std::move(p));
  }
  return out;
}

navigator_params WorldVehicles::navigator() const {
  const impl &d = *_impl;
  navigator_params n;
  n.ready = d.ready;
  if (!d.ready)
    return n;
  n.scale = kScale;
  n.tick = d.cfg.veh.dt;
  n.reach = d.cfg.reach_tol / kScale;
  const nav::veh_params &v = d.cfg.veh;
  n.rr = v.rr;
  n.d_hat = v.d_hat;
  n.dt = v.dt;
  n.vmax = v.vmax;
  n.L = v.L;
  n.delta_max = v.delta_max;
  n.a_max = v.a_max;
  n.a_lat_max = v.a_lat_max;
  n.k_steer = v.k_steer;
  n.nsub = v.nsub;
  n.allow_reverse = v.allow_reverse;
  n.stop_at_goal = d.cfg.stop_at_goal;
  n.park_hold = d.cfg.park_hold;
  return n;
}

const std::vector<float> &WorldVehicles::sdf() const { return _impl->sdf; }

std::vector<entity_info> WorldVehicles::entities() const {
  const impl &d = *_impl;
  std::vector<entity_info> out;
  for (const impl::entity &e : d.ents)
    out.push_back(d.info(e));
  // Two kinematic boxes make no contact in the physics world: they touch when they overlap. Only
  // boxes in the same or a neighbouring cell of a grid as wide as the largest box can overlap, so
  // each is tested against those alone: linear in the vehicles, where every pair was quadratic
  // (320,000 tests a call for 800, and the gym and the keys call this every tick).
  std::vector<std::size_t> kin;
  double cell = 0.0;
  for (std::size_t a = 0; a < out.size(); ++a)
    if (out[a].placed && (d.ents[a].posed || d.ents[a].plant == "kinematic")) {
      kin.push_back(a);
      const auto &h = d.ents[a].desc.half_extents;
      cell = std::max(cell, 2.0 * std::hypot(h.x, h.y));
    }
  if (kin.size() < 2 || !(cell > 0.0))
    return out;
  const auto key = [cell](double x, double y) {
    const auto ix = static_cast<std::int64_t>(std::floor(x / cell));
    const auto iy = static_cast<std::int64_t>(std::floor(y / cell));
    return (static_cast<std::uint64_t>(ix) << 32) ^ static_cast<std::uint32_t>(iy);
  };
  std::unordered_map<std::uint64_t, std::vector<std::size_t>> grid;
  grid.reserve(kin.size());
  for (const std::size_t a : kin)
    grid[key(out[a].x, out[a].y)].push_back(a);
  for (const std::size_t a : kin)
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy) {
        const auto it = grid.find(key(out[a].x + dx * cell, out[a].y + dy * cell));
        if (it == grid.end())
          continue;
        for (const std::size_t b : it->second)
          if (b > a && boxes_overlap(out[a], d.ents[a].desc, out[b], d.ents[b].desc))
            out[a].vehicle_contact = out[b].vehicle_contact = true;
      }
  return out;
}

std::size_t WorldVehicles::size() const { return _impl->ents.size(); }
std::uint64_t WorldVehicles::revision() const { return _impl->rev; }
const std::string &WorldVehicles::status() const { return _impl->status; }

std::string WorldVehicles::stats_lines() const {
  const impl &d = *_impl;
  // A summary fleet's quiet vehicles share its line, as in entities.summary.
  const std::set<std::uint64_t> keyed = d.keyed_ids();
  std::vector<int> quiet(d.groups.size(), 0);
  std::string out;
  for (const impl::entity &e : d.ents) {
    if (d.quiet(e, keyed)) {
      ++quiet[static_cast<std::size_t>(e.group)];
      continue;
    }
    const entity_info in = d.info(e);
    out +=
        (out.empty() ? "" : "\n") + std::to_string(e.id) + "  " + e.spec.name + "  " + e.plant +
        "  " +
        (in.placed ? fixed(in.speed, 1) + " m/s  " + fixed(in.power_kw, 1) + " kW  " +
                         fixed(in.energy_wh, 1) + " Wh  " + fixed(in.wheel_load / 1000.0, 1) + " kN"
                   : std::string("waiting"));
  }
  if (const std::string q = d.quiet_lines(quiet); !q.empty())
    out += (out.empty() ? "" : "\n") + q;
  return out;
}

std::vector<entity_wheel> WorldVehicles::wheels(std::uint64_t id) const {
  for (const impl::entity &e : _impl->ents)
    if (e.id == id)
      return _impl->wheels(e);
  return {};
}

} // namespace ariadne
} // namespace cvc
