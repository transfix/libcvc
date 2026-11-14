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
#include <cvc/ariadne/scene.h> // SceneAsset
#include <cvc/ariadne/world_vehicles.h>
#include <cvc/core/app.h>
#include <cvc/core/process.h> // self_path
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/policy_library.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/world_grid.h>
#include <cvc/physics/dyn_4w_plant.h>
#include <cvc/physics/dyn_bike_plant.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/world.h>
#include <cvc/state/state.h>
#include <exception>
#include <filesystem>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace cvc {
namespace ariadne {

namespace ph = cvc::physics;

namespace {

constexpr double kScale = 0.05;  // nav's normalized units per metre, as libcvc's nav demos
constexpr double kNavDt = 0.05;  // nav's tick (s), rounded to whole physics steps
constexpr double kReach = 3.0;   // m from its goal that counts as there
constexpr double kSpacing = 8.0; // m kept between spawns drawn from the grid
constexpr double kPi = 3.14159265358979323846;

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

} // namespace

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

ph::vehicle_desc vehicle_desc_of(const vehicle_archetype &a, double mass, double max_speed) {
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
  const std::string file = name == "humvee" ? std::string("Humvee.glb") : name;
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
                           std::vector<std::string> *warnings) {
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
    if (look.kind.empty()) { // the default: the Humvee model for a humvee, when it is installed
      const std::string model = v.preset == "humvee" ? find_vehicle_model("humvee") : "";
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
    if (look.kind == "file" && !doc_dir.empty() && relative(look.path))
      look.path = (fs::path(doc_dir) / look.path).string();
  }
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
  std::vector<float> sdf;
  std::vector<ph::body_id> ground;
  // Every fleet's navigator config: the largest and fastest vehicle any archetype may be (each
  // vehicle on a plant plans with its own envelope and size, nav_plant's plan_with_vehicle).
  cvc::nav::sim_world::config cfg;
  bool ready = false;      // cfg is the grid's
  bool stepping = false;   // before_step() began a step that after_step() ends
  int every = 1;           // physics steps per nav tick
  std::uint64_t steps = 0; // physics steps since the navigator was built
  std::uint64_t draws = 0; // goals drawn (each draw's seed)

  // A fleet: the entities on one plant, with a navigator of their own over the grid (a navigator
  // drives one plant). Their bodies are all in the one physics world, so they meet each other and
  // the world's bodies, and it steps them on its clock.
  struct fleet {
    std::string plant; // jolt, dynamic, dynamic4w or kinematic
    std::unique_ptr<cvc::nav::sim_world> sim;
    std::unique_ptr<ph::nav_plant> jolt;
    std::unique_ptr<ph::dyn_bike_plant> bike;
    std::unique_ptr<ph::dyn_4w_plant> four;
    // kinematic: each agent's box (by its index), and the poses it moves between over the tick's
    // steps (x, y of its centre, z, yaw): where the bicycle was, and where the tick took it
    std::vector<ph::body_id> boxes;
    std::vector<std::array<double, 4>> from, to;
    int sub = 0; // steps of the tick so far
    // The drive's external force on each agent (normalized), for the tick: the drive's threads
    // read it while sim->step runs.
    std::vector<std::array<float, 2>> force;
  };
  std::map<std::string, std::unique_ptr<fleet>> fleets; // by plant, made as entities need them

  struct entity {
    std::uint64_t id = 0;
    entity_spec spec;
    ph::vehicle_desc desc;
    std::string plant;                     // its archetype's
    std::uint64_t agent = 0;               // its fleet navigator's id (0: not placed)
    double sx = 0.0, sy = 0.0, syaw = 0.0; // where it starts (resolved at its first placing)
    bool started = false;
    double gx = 0.0, gy = 0.0; // its goal
    // Its intent keys as last read: its own, the intents table's by id, and by name.
    std::array<std::string, 3> seen_intent{no_intent_keys(), no_intent_keys(), no_intent_keys()};
    entity_intent applied;  // the intent the last navigator tick used
    std::string command_by; // who commanded it then: its controller, intent, blend or hold
    // The dynamic and kinematic plants' odometer (m), from its position at each navigator tick.
    double distance = 0.0, ox = 0.0, oy = 0.0;
    bool odometer = false;
  };
  std::vector<entity> ents;
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

  static void sample_force(void *user, int i, int, float, float, float *fx, float *fy) {
    const fleet *f = static_cast<const fleet *>(user);
    if (i >= 0 && static_cast<std::size_t>(i) < f->force.size()) {
      *fx = f->force[static_cast<std::size_t>(i)][0];
      *fy = f->force[static_cast<std::size_t>(i)][1];
    } else {
      *fx = *fy = 0.0f;
    }
  }

  fleet *fleet_of(const entity &e) const {
    const auto it = fleets.find(e.plant);
    return it == fleets.end() ? nullptr : it->second.get();
  }
  int index(const entity &e) const {
    const fleet *f = fleet_of(e);
    return f && f->sim && e.agent ? f->sim->index_of(e.agent) : -1;
  }
  double ground_height(double x, double y) const { return grid ? grid_height(*grid, x, y) : 0.0; }

  // Where a placed entity's chassis is: its body, the body's pose and its speed along its heading.
  struct pose {
    ph::body_id chassis;
    ph::vec3 position;
    ph::quat rotation;
    double yaw = 0.0, speed = 0.0;
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
    } else {
      if (f.bike) {
        p.chassis = f.bike->chassis(i);
        p.speed = f.bike->body(i).vx;
      } else if (f.four) {
        p.chassis = f.four->chassis(i);
        p.speed = f.four->body(i).vx;
      } else {
        p.chassis = f.boxes.at(static_cast<std::size_t>(i));
        p.speed = f.sim->speeds_normalized()[i] / kScale;
      }
      if (p.chassis.valid()) {
        const ph::body_state s = world->state(p.chassis);
        p.position = s.position;
        p.rotation = s.rotation;
      }
    }
    p.yaw = yaw_of(p.rotation);
    return p;
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
    const double wheel = d.axles.empty() ? 0.4 : d.axles.front().wheel_radius;
    return {cx, cy, ground_height(cx, cy) + d.half_extents.z + wheel, h};
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

  // The kinematic fleet's per-agent kinematics, from its vehicles' (after an agent comes or goes).
  void kinematics(fleet &f) {
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

  // The fleet on `plant`, made the first time an entity needs it.
  fleet &fleet_for(const std::string &plant) {
    std::unique_ptr<fleet> &slot = fleets[plant];
    if (slot)
      return *slot;
    auto f = std::make_unique<fleet>();
    f->plant = plant;
    f->sim = std::make_unique<cvc::nav::sim_world>(
        cfg, grid->occupancy.data(), grid->occupancy.data(), cvc::nav::coef_mlp::default_biased(),
        nullptr, nullptr, nullptr, 0);
    ph::nav_plant_options o;
    o.step_world = false; // the world steps on the clock; the plant commands and reads
    o.ground_bodies = ground;
    std::shared_ptr<const cvc::nav::world_grid> g = grid;
    o.ground_height = [g](double x, double y) { return grid_height(*g, x, y); };
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
    } // kinematic: nav's bicycle, and a kinematic box each in the world
    slot = std::move(f);
    return *slot;
  }

  // An entity out of its fleet: its vehicle (or box) out of the world.
  void remove_from_fleet(entity &e) {
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
      kinematics(*f);
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
    const double fixed_dt = world->config().fixed_dt;
    every = std::max(1, static_cast<int>(std::lround(kNavDt / fixed_dt)));
    const double dt = every * fixed_dt;
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
    cfg.freeze_sense = true; // the world's grid is known
    ready = true;
  }

  // Whether a placed vehicle other than `self` is within r metres of (x, y).
  bool crowded(double x, double y, double r, const entity *self) const {
    for (const entity &o : ents) {
      if (&o == self)
        continue;
      const fleet *f = fleet_of(o);
      const int i = index(o);
      if (!f || i < 0)
        continue;
      const pose p = pose_of(*f, i);
      if (std::hypot(p.position.x - x, p.position.y - y) < r)
        return true;
    }
    return false;
  }

  // Put entity `e` in the world: at its start if that is free, else at a drawn point. "" or why.
  std::string place(entity &e) {
    if (!ready)
      return "waiting for the world's grid";
    const double clearance = std::hypot(e.desc.half_extents.x, e.desc.half_extents.y) + 0.5;
    if (!e.started) {
      double px = 0.0, py = 0.0, ph = 0.0;
      if (e.spec.placed && free_at(e.spec.x, e.spec.y, 0.0)) {
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
    } else if (!free_at(e.sx, e.sy, 0.0)) { // a new world where its start is blocked
      const std::array<double, 2> p = draw_point(clearance, &e);
      e.sx = p[0];
      e.sy = p[1];
    }
    if (e.spec.controller == "nav@1") {
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
      fleet &f = fleet_for(e.plant);
      made = &f;
      if (f.jolt)
        a.vehicle = f.jolt->add_desc(e.desc);
      else if (f.bike)
        a.vehicle = f.bike->add_desc(e.desc);
      else if (f.four)
        a.vehicle = f.four->add_desc(e.desc);
      if (e.spec.controller == "nav@1" && !e.spec.policy.empty()) {
        cvc::nav::policy_library::entry p =
            policies.get(cvc::nav::policy_ref{e.spec.policy, e.spec.policy_sha256});
        a.policy = p.model;
        a.policy_key = p.key;
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
        kinematics(f);
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

  // Every entity out of the navigators (its vehicle out of the world, unless the world is gone).
  void unplace_all(bool world_gone) {
    if (world_gone) { // the vehicles went with the old world: forget them, don't remove them
      for (auto &[name, f] : fleets) {
        if (f->bike)
          f->bike->world_gone();
        if (f->four)
          f->four->world_gone();
        f->boxes.clear();
      }
      for (entity &e : ents)
        e.agent = 0;
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
    }
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
      apply_intents(*f, now);
      if (f->plant == "kinematic")
        for (const entity &e : ents)
          if (fleet_of(e) == f.get())
            if (const int i = index(e); i >= 0)
              f->from[static_cast<std::size_t>(i)] = bicycle_box(*f, i, e.desc);
      f->sim->step(1);
      if (f->plant == "kinematic") {
        for (const entity &e : ents)
          if (fleet_of(e) == f.get())
            if (const int i = index(e); i >= 0)
              f->to[static_cast<std::size_t>(i)] = bicycle_box(*f, i, e.desc);
        f->sub = 0;
      }
    }
    for (entity &e : ents) {
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
      // nav@1 wanders: one at its goal draws another.
      if (e.spec.controller == "nav@1" &&
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
    const fleet *f = fleet_of(e);
    const int i = index(e);
    if (!f || i < 0)
      return out;
    out.placed = true;
    const pose p = pose_of(*f, i);
    out.chassis = p.chassis;
    out.x = p.position.x;
    out.y = p.position.y;
    out.z = p.position.z;
    out.yaw = p.yaw;
    out.speed = p.speed;
    out.ground_z = ground_height(out.x, out.y);
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
    out.reached = std::hypot(e.gx - out.x, e.gy - out.y) < kReach + e.desc.half_extents.x;
    out.intent = e.applied;
    return out;
  }

  // Its wheels in its chassis box's frame, as drawn.
  std::vector<entity_wheel> wheels(const entity &e) const {
    std::vector<entity_wheel> out;
    const fleet *f = fleet_of(e);
    const int i = index(e);
    if (!f || i < 0)
      return out;
    const pose p = pose_of(*f, i);
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
    if (f->four) {
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
    if (f->jolt) {
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

  bool free_at(double x, double y, double clearance) const {
    if (!grid || grid->occupied(x, y))
      return false;
    int r = 0, c = 0;
    grid->sample_of(x, y, r, c);
    return sdf.empty() || sdf[static_cast<std::size_t>(r) * grid->cols + c] >= clearance;
  }
  // A free point at least `clearance` from anything blocked, kept kSpacing from the placed
  // entities' starts (when it can be), drawn the same way on every platform.
  std::array<double, 2> draw_point(double clearance, const entity *self) {
    std::array<double, 2> best{grid->center_x(), grid->center_y()};
    for (int attempt = 0; attempt < 24; ++attempt) {
      const std::vector<std::array<double, 2>> p =
          cvc::nav::free_points(*grid, sdf, 0x9e3779b97f4a7c15ull ^ (++draws), 1, clearance, 0.0);
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

  void publish_entities() {
    if (ekey.empty() || !root())
      return;
    std::set<std::string> now;
    std::string ids, summary;
    for (const entity &e : ents) {
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
  d.spawns->set_world(world_name, grid, std::move(world));
  d.grid = std::move(grid);
  d.ground = std::move(ground_bodies);
  d.status = d.grid ? "ready" : "waiting for the world's grid";
  try {
    d.build_navigator();
  } catch (const std::exception &ex) {
    d.fleets.clear();
    d.ready = false;
    d.status = std::string("the navigator could not be made: ") + ex.what();
  }
  d.place_all();
  d.put(d.vkey + ".status", d.status);
}

void WorldVehicles::world_reset() {
  impl &d = *_impl;
  d.unplace_all(true);
  try {
    d.build_navigator();
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
  if (!vehicle_plant_known(a->plant))
    return refuse("vehicle '" + a->id + "': plant '" + a->plant + "' is not one");
  impl::entity e;
  e.id = d.next_id;
  e.spec = spec;
  e.plant = a->plant;
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
  d.ents.push_back(std::move(e));
  impl::entity &placed = d.ents.back();
  if (d.ready) {
    if (std::string w = d.place(placed); !w.empty()) {
      d.ents.pop_back();
      return refuse(w);
    }
  }
  ++d.next_id;
  ++d.rev;
  {
    std::lock_guard<std::mutex> lock(d.mu);
    d.live.insert(placed.id);
  }
  return placed.id;
}

bool WorldVehicles::despawn(std::uint64_t id) {
  impl &d = *_impl;
  for (auto it = d.ents.begin(); it != d.ents.end(); ++it)
    if (it->id == id) {
      d.remove_from_fleet(*it);
      d.ents.erase(it);
      ++d.rev;
      std::lock_guard<std::mutex> lock(d.mu);
      d.live.erase(id);
      d.intents.erase(id);
      return true;
    }
  return false;
}

void WorldVehicles::clear() {
  impl &d = *_impl;
  d.unplace_all(false);
  d.ents.clear();
  ++d.rev;
  std::lock_guard<std::mutex> lock(d.mu);
  d.live.clear();
  d.intents.clear();
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
const std::string &WorldVehicles::intents_key() const { return _impl->ikey; }

WorldSpawns &WorldVehicles::spawns() { return *_impl->spawns; }
const WorldSpawns &WorldVehicles::spawns() const { return *_impl->spawns; }

void WorldVehicles::poll() {
  impl &d = *_impl;
  d.spawns->poll();
  if (!d.root() || d.vkey.empty())
    return;
  const std::string sel = d.get("selected") + "|" + d.get("spawn.mass") + "|" +
                          d.get("spawn.mass_t") + "|" + d.get("spawn.max_speed") + "|" +
                          d.get("spawn.max_speed_t");
  if (sel != d.seen_sel) {
    d.seen_sel = sel;
    d.publish_selection();
  }
  if (const std::string r = d.get("spawn.request"); r != d.seen_spawn) {
    d.seen_spawn = r;
    std::string why;
    const entity_spec s = d.spec_from_keys();
    const std::uint64_t id = spawn(s, &why);
    d.put(d.vkey + ".spawn.status",
          id ? "spawned " + std::to_string(id) + ": " + d.ents.back().spec.name : why);
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

void WorldVehicles::before_step() {
  impl &d = *_impl;
  d.stepping = false;
  if (!d.ready || d.ents.empty())
    return;
  if (d.steps++ % static_cast<std::uint64_t>(d.every) == 0)
    d.navigator_tick();
  d.before_world_step();
  d.stepping = true;
}

void WorldVehicles::after_step() {
  impl &d = *_impl;
  if (d.stepping)
    d.after_world_step();
  d.stepping = false;
}

void WorldVehicles::publish() { _impl->publish_entities(); }

std::vector<entity_info> WorldVehicles::entities() const {
  std::vector<entity_info> out;
  for (const impl::entity &e : _impl->ents)
    out.push_back(_impl->info(e));
  return out;
}

std::size_t WorldVehicles::size() const { return _impl->ents.size(); }
std::uint64_t WorldVehicles::revision() const { return _impl->rev; }
const std::string &WorldVehicles::status() const { return _impl->status; }

std::string WorldVehicles::stats_lines() const {
  std::string out;
  for (const impl::entity &e : _impl->ents) {
    const entity_info in = _impl->info(e);
    out +=
        (out.empty() ? "" : "\n") + std::to_string(e.id) + "  " + e.spec.name + "  " + e.plant +
        "  " +
        (in.placed ? fixed(in.speed, 1) + " m/s  " + fixed(in.power_kw, 1) + " kW  " +
                         fixed(in.energy_wh, 1) + " Wh  " + fixed(in.wheel_load / 1000.0, 1) + " kN"
                   : std::string("waiting"));
  }
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
