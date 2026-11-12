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
#include <cvc/ariadne/loader.h> // register_ari_block, LoadResult
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/value.h>
#include <cvc/ariadne/world_rules.h>
#include <cvc/state/state.h> // isValidStateName, SEPARATOR
#include <initializer_list>
#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {

namespace {

constexpr double kRoomQuantum = 1.0 / 120.0; // the holon's fixed_dt (world_clock's default)

bool finite_positive(double v) { return std::isfinite(v) && v > 0.0; }
bool finite_nonnegative(double v) { return std::isfinite(v) && v >= 0.0; }

std::string num_text(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%g", v);
  return buf;
}

// A controller id: not empty, and no state separator (the wire keeps ids as values, so `walker@1`
// is fine).
bool valid_controller_id(const std::string &id) {
  return !id.empty() && id.find(cvc::state::SEPARATOR) == std::string::npos &&
         id.find_first_of(" \t\r\n") == std::string::npos;
}

void warn(std::vector<std::string> *warnings, std::string w) {
  if (warnings)
    warnings->push_back(std::move(w));
}

// A number, or "" and the error in WHY.
bool number(const Value &v, const std::string &where, double &out, std::string &why) {
  out = v.as_double(std::numeric_limits<double>::quiet_NaN());
  if (std::isnan(out)) {
    why = where + " expects a number";
    return false;
  }
  return true;
}

bool boolean(const Value &v, const std::string &where, bool &out, std::string &why) {
  constexpr std::string_view yes[] = {"true", "yes", "on", "1"};
  constexpr std::string_view no[] = {"false", "no", "off", "0"};
  const auto among = [&](const auto &words) {
    return v.is_scalar() &&
           std::find(std::begin(words), std::end(words), v.scalar) != std::end(words);
  };
  if (among(yes)) {
    out = true;
    return true;
  }
  if (among(no)) {
    out = false;
    return true;
  }
  why = where + " expects true or false";
  return false;
}

// One name or a list of names.
bool names(const Value &v, const std::string &where, std::vector<std::string> &out,
           std::string &why) {
  out.clear();
  if (v.is_scalar()) {
    if (!v.empty())
      out.push_back(v.scalar);
    return true;
  }
  if (!v.is_seq()) {
    why = where + " expects a list of names";
    return false;
  }
  for (const Value &item : v.items) {
    if (!item.is_scalar()) {
      why = where + " expects names";
      return false;
    }
    out.push_back(item.scalar);
  }
  return true;
}

std::string parse_body(const Value &v, const std::string &where, avatar_body &body,
                       std::vector<std::string> *warnings) {
  if (!v.is_map())
    return where + " expects a map (mass:, width:, height:)";
  std::string why;
  for (const auto &[key, val] : v.entries) {
    double *field = key == "mass"     ? &body.mass
                    : key == "width"  ? &body.width
                    : key == "height" ? &body.height
                                      : nullptr;
    if (!field) {
      warn(warnings, where + " unknown key '" + key + "', ignored");
      continue;
    }
    if (!number(val, where + " " + key + ":", *field, why))
      return why;
  }
  return std::string();
}

std::string parse_archetypes(const Value &v, std::vector<avatar_archetype> &out,
                             std::vector<std::string> *warnings) {
  const std::string where = "world: avatars: archetypes:";
  if (!v.is_map() || v.entries.empty())
    return where + " expects a map of archetypes by id";
  out.clear();
  std::string why;
  for (const auto &[id, a] : v.entries) {
    avatar_archetype arch;
    arch.id = id;
    const std::string at = where + " " + id + ":";
    if (!a.is_map())
      return at + " expects a map (body:, controllers:)";
    for (const auto &[key, val] : a.entries) {
      if (key == "body") {
        if (std::string e = parse_body(val, at + " body:", arch.body, warnings); !e.empty())
          return e;
      } else if (key == "controllers") {
        if (!names(val, at + " controllers:", arch.controllers, why))
          return why;
      } else {
        warn(warnings, at + " unknown key '" + key + "', ignored");
      }
    }
    out.push_back(std::move(arch));
  }
  return std::string();
}

// The overrides a `vehicle:` map may set on its preset.
bool vehicle_override_key(const std::string &k) {
  return k == "mass" || k == "max_speed" || k == "max_accel" || k == "max_decel" ||
         k == "max_steer";
}

std::string parse_vehicle_archetype(const std::string &id, const Value &a, vehicle_archetype &v,
                                    std::vector<std::string> *warnings) {
  const std::string at = "world: vehicles: archetypes: " + id + ":";
  v.id = id;
  if (!a.is_map())
    return at + " expects a map (plant:, vehicle:, look:, ranges:, controllers:)";
  std::string why;
  for (const auto &[key, val] : a.entries) {
    if (key == "plant") {
      v.plant = val.as_string();
    } else if (key == "vehicle") {
      if (val.is_scalar()) {
        v.preset = val.scalar;
      } else if (val.is_map()) {
        for (const auto &[k, x] : val.entries) {
          if (k == "preset") {
            v.preset = x.as_string();
          } else if (vehicle_override_key(k)) {
            double n = 0.0;
            if (!number(x, at + " vehicle: " + k + ":", n, why))
              return why;
            v.overrides[k] = n;
          } else {
            warn(warnings, at + " vehicle: unknown key '" + k + "', ignored");
          }
        }
      } else {
        return at + " vehicle: expects a preset or a map (preset:, mass:, max_speed:, ...)";
      }
    } else if (key == "look") {
      if (!val.is_map())
        return at + " look: expects a map (box:, file:, asset:, model:, fit:, scale:, yaw:, z:)";
      for (const auto &[k, x] : val.entries) {
        if (k == "box") {
          v.look.kind = "box";
        } else if (k == "file" || k == "asset") {
          v.look.kind = k;
          v.look.path = x.as_string();
        } else if (k == "model") {
          v.look.kind = "model";
          v.look.path = x.as_string();
          v.look.fit = true;
        } else if (k == "fit") {
          if (!boolean(x, at + " look: fit:", v.look.fit, why))
            return why;
        } else if (k == "scale" || k == "yaw" || k == "z") {
          double n = 0.0;
          if (!number(x, at + " look: " + k + ":", n, why))
            return why;
          (k == "scale" ? v.look.scale : k == "yaw" ? v.look.yaw : v.look.z) = n;
        } else {
          warn(warnings, at + " look: unknown key '" + k + "', ignored");
        }
      }
    } else if (key == "ranges") {
      if (!val.is_map())
        return at + " ranges: expects a map (mass: [lo, hi], max_speed: [lo, hi])";
      for (const auto &[k, x] : val.entries) {
        if (k != "mass" && k != "max_speed") {
          warn(warnings, at + " ranges: unknown key '" + k + "', ignored");
          continue;
        }
        double lo = 0.0, hi = 0.0;
        if (!x.is_seq() || x.items.size() != 2 ||
            !number(x.items[0], at + " ranges: " + k + ":", lo, why) ||
            !number(x.items[1], at + " ranges: " + k + ":", hi, why))
          return at + " ranges: " + k + ": expects [lo, hi]";
        v.ranges[k] = {lo, hi};
      }
    } else if (key == "controllers") {
      if (!names(val, at + " controllers:", v.controllers, why))
        return why;
    } else {
      warn(warnings, at + " unknown key '" + key + "', ignored");
    }
  }
  return std::string();
}

std::string parse_vehicles(const Value &v, std::vector<vehicle_archetype> &out,
                           std::vector<std::string> *warnings) {
  if (!v.is_map())
    return "world: vehicles: expects a map (archetypes:)";
  for (const auto &[key, val] : v.entries) {
    if (key != "archetypes") {
      warn(warnings, "world: vehicles: unknown key '" + key + "', ignored");
      continue;
    }
    if (!val.is_map() || val.entries.empty())
      return "world: vehicles: archetypes: expects a map of archetypes by id";
    out.clear();
    for (const auto &[id, a] : val.entries) {
      vehicle_archetype arch;
      if (std::string e = parse_vehicle_archetype(id, a, arch, warnings); !e.empty())
        return e;
      out.push_back(std::move(arch));
    }
  }
  return std::string();
}

std::string parse_spawns(const Value &v, spawn_rules &out, std::vector<std::string> *warnings) {
  const std::string where = "world: spawns:";
  if (!v.is_map())
    return where + " expects a map (policy:, scope:, points:)";
  std::string why;
  for (const auto &[key, val] : v.entries) {
    if (key == "policy") {
      out.policy = val.as_string();
    } else if (key == "scope") {
      out.scope = val.as_string();
    } else if (key == "points") {
      if (!val.is_seq())
        return where + " points: expects a list of points";
      out.points.clear();
      for (const Value &item : val.items) {
        const std::string at = where + " points: [" + std::to_string(out.points.size()) + "]";
        if (!item.is_map())
          return at + " expects a map (at:, heading:, team:, archetype:, world:)";
        spawn_point p;
        bool placed = false;
        for (const auto &[k, x] : item.entries) {
          if (k == "at") {
            if (!x.is_seq() || x.items.size() < 2 || !number(x.items[0], at + " at:", p.x, why) ||
                !number(x.items[1], at + " at:", p.y, why))
              return at + " at: expects [x, y]";
            placed = true;
          } else if (k == "heading") {
            if (!number(x, at + " heading:", p.heading, why))
              return why;
          } else if (k == "team") {
            double t = 0.0;
            if (!number(x, at + " team:", t, why))
              return why;
            p.team = static_cast<int>(t);
          } else if (k == "archetype") {
            p.archetype = x.as_string();
          } else if (k == "world") {
            p.world = x.as_string();
          } else {
            warn(warnings, at + " unknown key '" + k + "', ignored");
          }
        }
        if (!placed)
          return at + " has no at: [x, y]";
        out.points.push_back(std::move(p));
      }
    } else {
      warn(warnings, where + " unknown key '" + key + "', ignored");
    }
  }
  return std::string();
}

std::string parse_energy(const Value &v, avatar_energy_policy &e,
                         std::vector<std::string> *warnings) {
  const std::string where = "world: avatars: policy: energy:";
  if (!v.is_map())
    return where + " expects a map";
  std::string why;
  for (const auto &[key, val] : v.entries) {
    const std::string at = where + " " + key + ":";
    double *field = key == "force_max"          ? &e.force_max
                    : key == "power_max"        ? &e.power_max
                    : key == "power_rest"       ? &e.power_rest
                    : key == "reserve_max"      ? &e.reserve_max
                    : key == "transport_cost"   ? &e.transport_cost
                    : key == "recover_fraction" ? &e.recover_fraction
                                                : nullptr;
    if (field) {
      if (!number(val, at, *field, why))
        return why;
    } else if (key == "enabled") {
      if (!boolean(val, at, e.enabled, why))
        return why;
    } else if (key == "empty") {
      const std::string s = val.as_string();
      if (s == "rest")
        e.empty = avatar_energy_policy::when_empty::rest;
      else if (s == "stop")
        e.empty = avatar_energy_policy::when_empty::stop;
      else
        return at + " expects rest or stop";
    } else {
      warn(warnings, where + " unknown key '" + key + "', ignored");
    }
  }
  return std::string();
}

std::string parse_policy(const Value &v, avatar_policy &p, std::vector<std::string> *warnings) {
  const std::string where = "world: avatars: policy:";
  if (!v.is_map())
    return where + " expects a map (max_speed:, allow_vertical:, energy:)";
  std::string why;
  for (const auto &[key, val] : v.entries) {
    if (key == "max_speed") {
      if (!number(val, where + " max_speed:", p.max_speed, why))
        return why;
    } else if (key == "allow_vertical") {
      if (!boolean(val, where + " allow_vertical:", p.allow_vertical, why))
        return why;
    } else if (key == "energy") {
      if (std::string e = parse_energy(val, p.energy, warnings); !e.empty())
        return e;
    } else {
      warn(warnings, where + " unknown key '" + key + "', ignored");
    }
  }
  return std::string();
}

// The kernel and version an id like `walker@1` names.
void split_id(const std::string &id, std::string &kernel, int &version) {
  const std::size_t at = id.find('@');
  kernel = id.substr(0, at);
  version = 1;
  if (at != std::string::npos) {
    const std::string v = id.substr(at + 1);
    char *end = nullptr;
    const long n = std::strtol(v.c_str(), &end, 10);
    if (!v.empty() && end && *end == '\0' && n > 0)
      version = static_cast<int>(n);
  }
}

std::string parse_controllers(const Value &v, std::vector<avatar_controller> &out,
                              std::vector<std::string> *warnings) {
  const std::string where = "world: controllers:";
  if (!v.is_map())
    return where + " expects a map of controllers by id";
  std::string why;
  for (const auto &[id, c] : v.entries) {
    const std::string at = where + " " + id + ":";
    avatar_controller ctl;
    ctl.id = id;
    split_id(id, ctl.kernel, ctl.version);
    ctl.params.clear();
    if (!c.is_map() && !c.empty())
      return at + " expects a map (kernel:, version:, params:)";
    for (const auto &[key, val] : c.entries) {
      if (key == "kernel") {
        ctl.kernel = val.as_string();
      } else if (key == "version") {
        double n = 0.0;
        if (!number(val, at + " version:", n, why))
          return why;
        if (!(n >= 1.0) || n != std::floor(n) || n > 1e6)
          return at + " version: expects a whole number from 1";
        ctl.version = static_cast<int>(n);
      } else if (key == "params") {
        if (!val.is_map() && !val.empty())
          return at + " params: expects a map of numbers";
        for (const auto &[name, pv] : val.entries) {
          double x = 0.0;
          if (!number(pv, at + " params: " + name + ":", x, why))
            return why;
          ctl.params[name] = x;
        }
      } else {
        warn(warnings, at + " unknown key '" + key + "', ignored");
      }
    }
    bool replaced = false;
    for (avatar_controller &have : out)
      if (have.id == ctl.id) {
        have = ctl;
        replaced = true;
      }
    if (!replaced)
      out.push_back(std::move(ctl));
  }
  return std::string();
}

} // namespace

double avatar_controller::param(const std::string &name, double def) const {
  const auto it = params.find(name);
  return it == params.end() ? def : it->second;
}

const avatar_archetype &world_rules::avatar() const {
  return archetypes.empty() ? fallback_avatar : archetypes.front();
}

const avatar_controller *world_rules::find_controller(const std::string &id) const {
  for (const avatar_controller &c : controllers)
    if (c.id == id)
      return &c;
  return nullptr;
}

const avatar_controller &world_rules::controller() const {
  const avatar_archetype &a = avatar();
  const avatar_controller *c = a.controllers.empty() ? nullptr : find_controller(a.controllers[0]);
  return c ? *c : fallback_controller;
}

const vehicle_archetype *world_rules::find_vehicle(const std::string &id) const {
  for (const vehicle_archetype &v : vehicles)
    if (v.id == id)
      return &v;
  return nullptr;
}

std::vector<std::string> vehicle_presets() { return {"humvee", "roboracer"}; }

bool vehicle_controller_known(const std::string &id) { return id == "nav@1" || id == "none"; }

bool spawn_policy_known(const std::string &policy) {
  return policy == "fixed" || policy == "random" || policy == "round_robin" ||
         policy == "nearest_free";
}

bool spawn_scope_known(const std::string &scope) {
  return scope == "team" || scope == "archetype" || scope == "all";
}

bool vehicle_plant_known(const std::string &plant) {
  return plant == "jolt" || plant == "kinematic" || plant == "dynamic" || plant == "dynamic4w";
}

world_rules default_world_rules() { return world_rules{}; }

bool controller_kernel_known(const std::string &kernel, int version) {
  return kernel == "walker" && version == 1;
}

std::string check_world_rules(const world_rules &r) {
  const cvc::physics::vec3 &g = r.physics.gravity;
  if (!std::isfinite(g.x) || !std::isfinite(g.y) || !std::isfinite(g.z))
    return "physics: gravity must be finite";

  std::set<std::string> ids;
  for (const avatar_controller &c : r.controllers) {
    const std::string at = "controller '" + c.id + "'";
    if (!valid_controller_id(c.id))
      return at + ": an id must be a name without '" + std::string(cvc::state::SEPARATOR) +
             "' or spaces";
    if (!ids.insert(c.id).second)
      return at + " is defined twice";
    if (!controller_kernel_known(c.kernel, c.version))
      return at + ": this build has no kernel " + c.kernel + "@" + std::to_string(c.version) +
             " (it runs walker@1)";
    for (const auto &[name, v] : c.params) {
      if (!cvc::state::isValidStateName(name))
        return at + ": param '" + name + "' is not a name";
      if (!std::isfinite(v))
        return at + ": param '" + name + "' must be finite";
    }
  }

  if (r.archetypes.empty())
    return "there must be an avatar archetype";
  std::set<std::string> archetypes;
  for (const avatar_archetype &a : r.archetypes) {
    const std::string at = "archetype '" + a.id + "'";
    if (!cvc::state::isValidStateName(a.id))
      return at + ": an id must be a name (letters, digits and _)";
    if (!archetypes.insert(a.id).second)
      return at + " is defined twice";
    if (!finite_positive(a.body.mass) || !finite_positive(a.body.width) ||
        !finite_positive(a.body.height))
      return at + ": body mass, width and height must be positive";
    if (a.controllers.empty())
      return at + " allows no controller";
    for (const std::string &c : a.controllers)
      if (!ids.count(c))
        return at + " allows controller '" + c + "', which controllers: does not define";
  }

  std::set<std::string> vehicles;
  for (const vehicle_archetype &v : r.vehicles) {
    const std::string at = "vehicle '" + v.id + "'";
    if (!cvc::state::isValidStateName(v.id))
      return at + ": an id must be a name (letters, digits and _)";
    if (!vehicles.insert(v.id).second)
      return at + " is defined twice";
    if (!vehicle_plant_known(v.plant))
      return at + ": plant '" + v.plant + "' is not one of jolt, kinematic, dynamic, dynamic4w";
    bool preset = false;
    for (const std::string &p : vehicle_presets())
      preset = preset || p == v.preset;
    if (!preset)
      return at + ": vehicle '" + v.preset + "' is not a preset (humvee, roboracer)";
    for (const auto &[k, x] : v.overrides)
      if (k == "max_speed" ? !finite_nonnegative(x) : !finite_positive(x))
        return at + ": vehicle: " + k + " must be " +
               (k == "max_speed" ? "zero or more" : "positive");
    if (!v.look.kind.empty() && v.look.kind != "box" && v.look.path.empty())
      return at + ": look: " + v.look.kind + " names nothing";
    if (!finite_positive(v.look.scale) || !std::isfinite(v.look.yaw) || !std::isfinite(v.look.z))
      return at + ": look: scale must be positive, yaw and z finite";
    for (const auto &[k, lohi] : v.ranges)
      if (!finite_positive(lohi.first) || !std::isfinite(lohi.second) || lohi.second < lohi.first)
        return at + ": ranges: " + k + " must be [lo, hi] with 0 < lo <= hi";
    if (v.controllers.empty())
      return at + " allows no controller";
    for (const std::string &c : v.controllers)
      if (!vehicle_controller_known(c))
        return at + " allows controller '" + c + "'; a vehicle has nav@1 or none";
  }

  if (!spawn_policy_known(r.spawns.policy))
    return "spawns: policy '" + r.spawns.policy +
           "' is not one of fixed, random, round_robin, nearest_free";
  if (!spawn_scope_known(r.spawns.scope))
    return "spawns: scope '" + r.spawns.scope + "' is not one of team, archetype, all";
  for (const spawn_point &sp : r.spawns.points) {
    if (!std::isfinite(sp.x) || !std::isfinite(sp.y) || !std::isfinite(sp.heading))
      return "spawns: a point's at: and heading: must be finite";
    if (!sp.archetype.empty() && !vehicles.count(sp.archetype))
      return "spawns: a point names vehicle '" + sp.archetype + "', which vehicles: does not offer";
  }

  const avatar_policy &p = r.policy;
  if (!finite_nonnegative(p.max_speed))
    return "policy: max_speed must be zero or more";
  const avatar_energy_policy &e = p.energy;
  for (const auto &[name, v] :
       std::initializer_list<std::pair<const char *, double>>{{"force_max", e.force_max},
                                                              {"power_max", e.power_max},
                                                              {"power_rest", e.power_rest},
                                                              {"reserve_max", e.reserve_max},
                                                              {"transport_cost", e.transport_cost}})
    if (!finite_nonnegative(v))
      return std::string("policy: energy: ") + name + " must be zero or more";
  if (!(e.recover_fraction >= 0.0 && e.recover_fraction <= 1.0))
    return "policy: energy: recover_fraction must be from 0 to 1";
  return std::string();
}

std::string parse_world_block(const Value &content, world_rules &rules,
                              std::vector<std::string> *warnings) {
  if (content.is_scalar() && content.empty())
    return std::string(); // a bare `world:`
  if (!content.is_map())
    return "world: expects a map (avatars:, controllers:, vehicles:, spawns:)";
  // Controllers first: an archetype names them.
  if (const Value *c = content.find("controllers"))
    if (std::string e = parse_controllers(*c, rules.controllers, warnings); !e.empty())
      return e;
  for (const auto &[key, v] : content.entries) {
    if (key == "controllers")
      continue;
    if (key == "vehicles") {
      if (std::string e = parse_vehicles(v, rules.vehicles, warnings); !e.empty())
        return e;
      continue;
    }
    if (key == "spawns") {
      if (std::string e = parse_spawns(v, rules.spawns, warnings); !e.empty())
        return e;
      continue;
    }
    if (key != "avatars") {
      warn(warnings, "world: unknown key '" + key + "', ignored");
      continue;
    }
    if (!v.is_map())
      return "world: avatars: expects a map (archetypes:, policy:)";
    for (const auto &[akey, av] : v.entries) {
      if (akey == "archetypes") {
        if (std::string e = parse_archetypes(av, rules.archetypes, warnings); !e.empty())
          return e;
      } else if (akey == "policy") {
        if (std::string e = parse_policy(av, rules.policy, warnings); !e.empty())
          return e;
      } else if (akey == "catalog") {
        warn(warnings, "world: avatars: catalog: is not read yet; the relay seeds its default "
                       "catalog (ARIADNE_WORLDS.md WD6)");
      } else {
        warn(warnings, "world: avatars: unknown key '" + akey + "', ignored");
      }
    }
  }
  if (std::string e = check_world_rules(rules); !e.empty())
    return "world: " + e;
  return std::string();
}

void register_world_block() {
  static std::once_flag once;
  std::call_once(once, [] {
    register_ari_block("world", [](const Value &content, LoadResult &out) {
      out.extras.emplace_back("world", content);
    });
  });
}

const Value *find_world_block(const LoadResult &lr) {
  for (const auto &[key, content] : lr.extras)
    if (key == "world")
      return &content;
  return nullptr;
}

std::string world_rules_from(const Scene &scene, const Value *world, world_rules &out,
                             std::vector<std::string> *warnings) {
  world_rules r = default_world_rules();
  const ScenePhysics &ph = scene.physics;
  if (ph.present) {
    r.physics.gravity = cvc::physics::vec3{ph.gravity[0], ph.gravity[1], ph.gravity[2]};
    if (ph.fixed_dt != 0.0 && std::fabs(ph.fixed_dt - kRoomQuantum) > 1e-9)
      warn(warnings, "scene: physics: fixed_dt " + num_text(ph.fixed_dt) +
                         " is ignored: a room steps its avatars every 1/120 s");
    if (ph.backend != "auto" && ph.backend != "jolt")
      warn(warnings, "scene: physics: backend " + ph.backend +
                         " is ignored: a room's avatars are Jolt bodies when the build has Jolt");
    if (!ph.materials.empty())
      warn(warnings, "scene: physics: materials wait for the world runtime (ARIADNE_WORLDS.md "
                     "WD3); the room's statics keep the default material");
  }
  if (world)
    if (std::string e = parse_world_block(*world, r, warnings); !e.empty())
      return e;
  if (std::string e = check_world_rules(r); !e.empty())
    return e;
  out = std::move(r);
  return std::string();
}

} // namespace ariadne
} // namespace cvc
