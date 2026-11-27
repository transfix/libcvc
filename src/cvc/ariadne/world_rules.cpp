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
#include <cvc/core/sim_tick.h> // the room's rate (scene: clock: hz)
#include <cvc/state/state.h>   // isValidStateName, SEPARATOR
#include <initializer_list>
#include <limits>
#include <locale>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {

namespace {

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
    return where + " expects a map (mass:, width:, height:, density:)";
  std::string why;
  for (const auto &[key, val] : v.entries) {
    double *field = key == "mass"      ? &body.mass
                    : key == "width"   ? &body.width
                    : key == "height"  ? &body.height
                    : key == "density" ? &body.density
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
      return at + " expects a map (body:, controllers:, or vehicle:)";
    for (const auto &[key, val] : a.entries) {
      if (key == "body") {
        if (std::string e = parse_body(val, at + " body:", arch.body, warnings); !e.empty())
          return e;
      } else if (key == "controllers") {
        if (!names(val, at + " controllers:", arch.controllers, why))
          return why;
      } else if (key == "vehicle") {
        if (!val.is_scalar() || val.scalar.empty())
          return at + " vehicle: expects a vehicle archetype's id";
        arch.vehicle = val.scalar;
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
  bool named = false; // vehicle: named its preset
  for (const auto &[key, val] : a.entries) {
    if (key == "plant") {
      v.plant = val.as_string();
    } else if (key == "vehicle") {
      if (val.is_scalar()) {
        v.preset = val.scalar;
        named = true;
      } else if (val.is_map()) {
        for (const auto &[k, x] : val.entries) {
          if (k == "preset") {
            v.preset = x.as_string();
            named = true;
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
        return at + " look: expects a map (box:, file:, asset:, model:, data:, chassis:, wheel:, "
                    "rig:, fit:, texture:, scale:, yaw:, z:)";
      bool fit_given = false, rig_given = false;
      for (const auto &[k, x] : val.entries) {
        if (k == "chassis" || k == "wheel") {
          (k == "chassis" ? v.look.chassis : v.look.wheel) = x.as_string();
        } else if (k == "rig") {
          rig_given = true;
          if (x.is_map()) {
            if (!parse_vehicle_rig(x, v.look.rig, &why))
              return at + " look: rig: " + why;
          } else {
            v.look.rig_file = x.as_string();
          }
        } else if (k == "box") {
          v.look.kind = "box";
        } else if (k == "file" || k == "asset") {
          v.look.kind = k;
          v.look.path = x.as_string();
        } else if (k == "model") {
          v.look.kind = "model";
          v.look.path = x.as_string();
          v.look.fit = true;
        } else if (k == "data") {
          v.look.data = x.as_string();
        } else if (k == "fit") {
          fit_given = true;
          if (!boolean(x, at + " look: fit:", v.look.fit, why))
            return why;
        } else if (k == "texture") {
          if (!boolean(x, at + " look: texture:", v.look.texture, why))
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
      // A parted look: the chassis, the wheel and the rig, all three, in place of one model; on
      // disk, or with `data: NAME` in that entry's package. Fitted to the vehicle always.
      if (!v.look.chassis.empty() || !v.look.wheel.empty() || rig_given) {
        if (v.look.chassis.empty() || v.look.wheel.empty() ||
            (v.look.rig_file.empty() && v.look.rig.hubs.empty()))
          return at + " look: a parted look needs chassis:, wheel: and rig:";
        if (!v.look.kind.empty())
          return at + " look: chassis:, wheel: and rig: are a model in parts; drop " +
                 (v.look.kind == "box" ? std::string("box") : v.look.kind) + ":";
        v.look.kind = v.look.data.empty() ? "file" : "data";
        v.look.fit = true;
      } else if (!v.look.data.empty()) {
        // `data: NAME` with `file: FILE`: the file inside a data: entry's package, a model fitted
        // to the chassis unless `fit: false`.
        if (v.look.kind != "file" || v.look.path.empty())
          return at + " look: data: needs file: (the model's file in the package)";
        v.look.kind = "data";
        if (!fit_given)
          v.look.fit = true;
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
    } else if (key == "drivable") {
      if (!boolean(val, at + " drivable:", v.drivable, why))
        return why;
    } else if (key == "reach") {
      if (!number(val, at + " reach:", v.reach, why))
        return why;
      if (!(v.reach >= 0.0))
        return at + " reach: expects metres, 0 or more";
    } else if (key == "controls") {
      v.controls = val.as_string();
      if (v.controls != "car" && v.controls != "boat")
        return at + " controls: '" + v.controls + "' is not one (car, boat)";
    } else {
      warn(warnings, at + " unknown key '" + key + "', ignored");
    }
  }
  // dynamic:<preset>, dynamic4w:<preset> and dynamicboat:<preset> name the vehicle too, as
  // plant_host's names do.
  if (const std::size_t colon = v.plant.find(':'); colon != std::string::npos) {
    const std::string kind = v.plant.substr(0, colon), preset = v.plant.substr(colon + 1);
    if (kind != "dynamic" && kind != "dynamic4w" && kind != "dynamicboat")
      return at + " plant: '" + v.plant +
             "': only dynamic, dynamic4w and dynamicboat name a preset";
    if (named && preset != v.preset)
      return at + " plant: '" + v.plant + "' names preset '" + preset + "', vehicle: '" + v.preset +
             "'";
    v.plant = kind;
    v.preset = preset;
  }
  return std::string();
}

// world: vehicles: start: into `out`: "" or what is wrong. The archetypes are checked once the
// whole block is read.
std::string parse_vehicle_starts(const Value &v, std::vector<vehicle_start> &out,
                                 std::vector<std::string> *warnings) {
  const std::string where = "world: vehicles: start:";
  if (!v.is_seq())
    return where + " expects a list ({archetype:, count:, name:, team:, controller:, plant:, "
                   "policy:, at:, heading:} or a fleet's {fleet:, spawn:, seed:, rally:, colors:, "
                   "keys:, ...})";
  out.clear();
  std::string why;
  for (const Value &item : v.items) {
    const std::string at = where + " [" + std::to_string(out.size()) + "]";
    if (!item.is_map())
      return at + " expects a map (archetype:, count:, name:, team:, controller:, plant:, "
                  "policy:, at:, heading:)";
    vehicle_start s;
    for (const auto &[k, x] : item.entries) {
      if (k == "policy" && x.is_map()) {
        // A fleet's weights: { uri:, path:, sha256: } or { data: <entry> }.
        for (const auto &[pk, pv] : x.entries) {
          if (pk == "uri" || pk == "path" || pk == "sha256" || pk == "data")
            (pk == "uri"      ? s.policy
             : pk == "path"   ? s.policy_path
             : pk == "sha256" ? s.policy_sha256
                              : s.policy_data) = pv.as_string();
          else
            warn(warnings, at + " policy: unknown key '" + pk + "', ignored");
        }
        if (s.policy.empty() && s.policy_data.empty())
          return at + " policy: expects a uri: or a data: entry ({uri:, path:, sha256:} or "
                      "{data:})";
        if (!s.policy.empty() && !s.policy_data.empty())
          return at + " policy: names a uri: or a data: entry, not both";
      } else if (k == "plant" && (x.is_seq() || x.is_map())) {
        // A fleet's plants: a list its vehicles cycle over, or each team's ({ 0: jolt }).
        std::string text;
        if (x.is_seq())
          for (const Value &p : x.items)
            text += (text.empty() ? "" : ", ") + p.as_string();
        else
          for (const auto &[t, p] : x.entries)
            text += (text.empty() ? "" : ", ") + t + ": " + p.as_string();
        if (std::string e = parse_fleet_plants(text, s.plants); !e.empty())
          return at + " plant: " + e;
        s.plant = s.plants.cycle.empty() ? std::string() : s.plants.cycle.front();
      } else if (k == "archetype" || k == "name" || k == "controller" || k == "plant" ||
                 k == "policy") {
        (k == "archetype"    ? s.archetype
         : k == "name"       ? s.name
         : k == "controller" ? s.controller
         : k == "plant"      ? s.plant
                             : s.policy) = x.as_string();
      } else if (k == "fleet" || k == "spawn" || k == "colors" || k == "keys" || k == "sense" ||
                 k == "belief" || k == "metric") {
        (k == "fleet"    ? s.fleet
         : k == "spawn"  ? s.layout
         : k == "colors" ? s.colors
         : k == "keys"   ? s.keys
         : k == "sense"  ? s.sense
         : k == "belief" ? s.belief
                         : s.metric) = x.as_string();
      } else if (k == "collide") {
        if (!boolean(x, at + " collide:", s.collide, why))
          return why;
      } else if (k == "count" || k == "team" || k == "seed") {
        double n = 0.0;
        if (!number(x, at + " " + k + ":", n, why))
          return why;
        if (k == "team") {
          s.team = static_cast<int>(n);
        } else if (k == "seed") {
          if (n < 0.0 || n > 9007199254740992.0 || n != std::floor(n))
            return at + " seed: expects a whole number from 0";
          s.seed = static_cast<unsigned long long>(n);
        } else {
          // A fleet's cap; an unnamed entry's is checked once the entry is read (fleet: may come
          // after count:).
          if (n < 1.0 || n > kMaxFleetCount || n != std::floor(n))
            return at + " count: expects a whole number from 1 to " +
                   std::to_string(kMaxVehicleStartCount) + " (" + std::to_string(kMaxFleetCount) +
                   " for a fleet)";
          s.count = static_cast<int>(n);
        }
      } else if (k == "rally") {
        // rally: T, or rally: { count: T, assign: latitude }
        const Value *c = x.is_map() ? nullptr : &x;
        if (x.is_map())
          for (const auto &[rk, rv] : x.entries) {
            if (rk == "count")
              c = &rv;
            else if (rk == "assign")
              s.assign = rv.as_string();
            else
              warn(warnings, at + " rally: unknown key '" + rk + "', ignored");
          }
        double n = 0.0;
        if (!c || !number(*c, at + " rally:", n, why) || n < 1.0 || n > kMaxFleetRally ||
            n != std::floor(n))
          return at + " rally: expects a count from 1 to " + std::to_string(kMaxFleetRally) +
                 " ({count:, assign: latitude})";
        s.rally = static_cast<int>(n);
      } else if (k == "at") {
        if (!x.is_seq() || x.items.size() < 2 || !number(x.items[0], at + " at:", s.x, why) ||
            !number(x.items[1], at + " at:", s.y, why))
          return at + " at: expects [x, y]";
        s.placed = true;
      } else if (k == "heading") {
        if (!number(x, at + " heading:", s.heading, why))
          return why;
        s.has_heading = true;
      } else {
        warn(warnings, at + " unknown key '" + k + "', ignored");
      }
    }
    if (s.archetype.empty())
      return at + " names no archetype:";
    if (s.fleet.empty()) {
      if (s.count > kMaxVehicleStartCount)
        return at + " count: expects a whole number from 1 to " +
               std::to_string(kMaxVehicleStartCount) + " (name a fleet: for more)";
      for (const auto &[k, x] : item.entries)
        if (k == "spawn" || k == "seed" || k == "rally" || k == "colors" || k == "keys" ||
            k == "sense" || k == "belief" || k == "metric" || k == "collide" ||
            (k == "policy" && x.is_map()))
          warn(warnings, at + " " + k + ": is a fleet's (fleet: names one), ignored");
        else if (k == "plant" && (x.is_seq() || x.is_map()))
          warn(warnings,
               at + " plant: a list or each team's is a fleet's (fleet: names one); " +
                   (s.plant.empty() ? std::string("the archetype's") : "'" + s.plant + "'") +
                   " is used");
      const vehicle_start plain;
      s.metric = plain.metric;
      s.plants = plain.plants;
      s.layout = plain.layout;
      s.seed = plain.seed;
      s.rally = plain.rally;
      s.assign = plain.assign;
      s.colors = plain.colors;
      s.keys = plain.keys;
      s.sense = plain.sense;
      s.belief = plain.belief;
      if (!s.policy_path.empty() || !s.policy_sha256.empty() || !s.policy_data.empty()) {
        s.policy = plain.policy;
        s.policy_path = plain.policy_path;
        s.policy_sha256 = plain.policy_sha256;
        s.policy_data = plain.policy_data;
      }
    } else {
      if (!cvc::state::isValidStateName(s.fleet))
        return at + " fleet: '" + s.fleet + "' is not a state name";
      for (const vehicle_start &o : out)
        if (o.fleet == s.fleet)
          return at + " fleet: '" + s.fleet + "' is named twice";
      if (s.placed)
        return at + " fleet: '" + s.fleet + "' is laid out by spawn:, not at:";
      if (s.controller == "pose")
        return at + " fleet: '" + s.fleet + "' drives its vehicles; controller: pose places one";
      if (!s.layout.empty() && s.layout != "opposed" && s.layout != "scattered" &&
          s.layout != "points")
        return at + " spawn: '" + s.layout + "' is not one (opposed, scattered, points)";
      if (!s.assign.empty() && s.assign != "latitude")
        return at + " rally: assign: '" + s.assign + "' is not one (latitude)";
      if (!s.colors.empty() && s.colors != "hsv" && s.colors != "spawn")
        return at + " colors: '" + s.colors + "' is not one (hsv, spawn)";
      if (!s.keys.empty() && s.keys != "summary" && s.keys != "all")
        return at + " keys: '" + s.keys + "' is not one (summary, all)";
      if (!s.sense.empty() && s.sense != "known" && s.sense != "fog")
        return at + " sense: '" + s.sense + "' is not one (known, fog)";
      if (!s.belief.empty() && s.belief != "shared" && s.belief != "grouped" &&
          s.belief != "private")
        return at + " belief: '" + s.belief + "' is not one (shared, grouped, private)";
      if (!s.metric.empty() && !fleet_metric_known(s.metric))
        return at + " metric: '" + s.metric + "' is not one (trained, vehicle, policy)";
      if (!s.plants.teams.empty()) // its teams are its rally points (rally: may come after)
        if (std::string e = parse_fleet_plants(s.plants.text(), s.plants, s.rally); !e.empty())
          return at + " plant: " + e;
    }
    if (!s.plant.empty() && !vehicle_plant_known(s.plant))
      return at + " plant: '" + s.plant +
             "' is not one (jolt, kinematic, dynamic, dynamic4w, dynamicboat)";
    if (!s.controller.empty() && !vehicle_controller_known(s.controller))
      return at + " controller: '" + s.controller + "' is not one";
    out.push_back(std::move(s));
  }
  return std::string();
}

std::string parse_vehicles(const Value &v, std::vector<vehicle_archetype> &out,
                           std::string &intents, std::string &poses,
                           std::vector<vehicle_start> &starts, std::vector<std::string> *warnings) {
  if (!v.is_map())
    return "world: vehicles: expects a map (archetypes:, intents:, poses:, start:)";
  for (const auto &[key, val] : v.entries) {
    if (key == "start") {
      if (std::string e = parse_vehicle_starts(val, starts, warnings); !e.empty())
        return e;
      continue;
    }
    if (key == "intents" || key == "poses") {
      std::string &table = key == "intents" ? intents : poses;
      table = val.is_scalar() ? val.scalar : std::string();
      std::string path = !table.empty() && table[0] == '/' ? table.substr(1) : table;
      bool ok = !path.empty();
      for (std::size_t at = 0; ok && at <= path.size();) {
        const std::size_t dot = std::min(path.find('.', at), path.size());
        ok = cvc::state::isValidStateName(path.substr(at, dot - at));
        at = dot + 1;
      }
      if (!ok)
        return "world: vehicles: " + key +
               ": expects a state path (names joined by '.', a leading / for one outside the "
               "document's prefix)";
      continue;
    }
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
  for (std::size_t i = 0; i < starts.size(); ++i)
    if (std::none_of(out.begin(), out.end(),
                     [&](const vehicle_archetype &a) { return a.id == starts[i].archetype; }))
      return "world: vehicles: start: [" + std::to_string(i) + "] names archetype '" +
             starts[i].archetype + "', which archetypes: does not define";
  return std::string();
}

std::string parse_spawns(const Value &v, spawn_rules &out, std::vector<std::string> *warnings) {
  const std::string where = "world: spawns:";
  if (!v.is_map())
    return where + " expects a map (policy:, scope:, ford_depth:, keel_clearance:, "
                   "shore_clearance:, points:)";
  std::string why;
  for (const auto &[key, val] : v.entries) {
    if (key == "policy") {
      out.policy = val.as_string();
    } else if (key == "scope") {
      out.scope = val.as_string();
    } else if (key == "ford_depth" || key == "keel_clearance" || key == "shore_clearance") {
      double n = 0.0;
      if (!number(val, where + " " + key + ":", n, why))
        return why;
      (key == "ford_depth"       ? out.ford_depth
       : key == "keel_clearance" ? out.keel_clearance
                                 : out.shore_clearance) = n;
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

bool parse_vehicle_rig(const Value &v, vehicle_rig &out, std::string *why) {
  const auto fail = [&](const std::string &w) {
    if (why)
      *why = w;
    return false;
  };
  if (!v.is_map())
    return fail("expects a map (radius:, hubs:, steer:, mirror:, up:)");
  vehicle_rig r;
  const Value *hubs = v.find("hubs");
  const Value *list = hubs && hubs->is_map() ? hubs->find("xyz") : hubs;
  std::vector<std::string> names; // hubs: order:, for steer:
  if (hubs && hubs->is_map())
    if (const Value *order = hubs->find("order"); order && order->is_seq())
      for (const Value &n : order->items)
        names.push_back(n.as_string());
  if (!list || !list->is_seq() || list->items.empty())
    return fail("hubs: expects a list of hubs, each [x, y, z]");
  for (const Value &h : list->items) {
    std::array<double, 3> p{};
    bool ok = h.is_seq() && h.items.size() == 3;
    for (std::size_t k = 0; ok && k < 3; ++k) {
      p[k] = h.items[k].as_double(std::numeric_limits<double>::quiet_NaN());
      ok = std::isfinite(p[k]);
    }
    if (!ok)
      return fail("hubs: each hub is [x, y, z], three numbers");
    r.hubs.push_back(p);
  }
  const Value *radius = v.find("radius");
  if (!radius)
    if (const Value *w = v.find("wheel"); w && w->is_map())
      radius = w->find("radius");
  if (radius) {
    r.radius = radius->as_double(std::numeric_limits<double>::quiet_NaN());
    if (!finite_positive(r.radius))
      return fail("radius: must be a number > 0");
  }
  if (const Value *m = v.find("mirror")) {
    r.mirror = m->as_string();
    if (r.mirror != "turn" && r.mirror != "reflect" && r.mirror != "none")
      return fail("mirror: '" + r.mirror + "' is not one of turn, reflect, none");
  }
  if (const Value *u = v.find("up")) {
    const std::string up = u->as_string();
    if (up != "z" && up != "y")
      return fail("up: '" + up + "' is not z or y");
    r.y_up = up == "y";
  }
  r.steers.assign(r.hubs.size(), false);
  if (const Value *s = v.find("steer")) {
    if (!s->is_seq())
      return fail("steer: expects a list of hubs, by index or by name");
    for (const Value &e : s->items) {
      const std::string name = e.as_string();
      std::size_t k = r.hubs.size();
      for (std::size_t i = 0; i < names.size() && i < r.hubs.size(); ++i)
        if (names[i] == name)
          k = i;
      if (k == r.hubs.size()) {
        const double d = e.as_double(-1.0);
        if (d >= 0.0 && d == std::floor(d) && d < static_cast<double>(r.hubs.size()))
          k = static_cast<std::size_t>(d);
      }
      if (k == r.hubs.size())
        return fail("steer: '" + name + "' names no hub");
      r.steers[k] = true;
    }
  } else { // the front axle's: the hubs farthest forward (x; z in glTF's frame)
    const int fwd = r.y_up ? 2 : 0;
    double lo = std::numeric_limits<double>::infinity(), hi = -lo;
    for (const std::array<double, 3> &h : r.hubs) {
      lo = std::min(lo, h[fwd]);
      hi = std::max(hi, h[fwd]);
    }
    if (hi > lo)
      for (std::size_t i = 0; i < r.hubs.size(); ++i)
        r.steers[i] = r.hubs[i][fwd] >= hi - 0.05 * (hi - lo);
  }
  out = std::move(r);
  return true;
}

std::string vehicle_rig_text(const vehicle_rig &r) {
  std::ostringstream o;
  o.imbue(std::locale::classic());
  o.precision(17);
  o << "{";
  if (r.radius > 0.0)
    o << "radius: " << r.radius << ", ";
  o << "hubs: [";
  for (std::size_t i = 0; i < r.hubs.size(); ++i)
    o << (i ? ", [" : "[") << r.hubs[i][0] << ", " << r.hubs[i][1] << ", " << r.hubs[i][2] << "]";
  o << "], steer: [";
  bool first = true;
  for (std::size_t i = 0; i < r.steers.size(); ++i)
    if (r.steers[i]) {
      o << (first ? "" : ", ") << i;
      first = false;
    }
  o << "], mirror: " << r.mirror << ", up: " << (r.y_up ? "y" : "z") << "}";
  return o.str();
}

double avatar_controller::param(const std::string &name, double def) const {
  const auto it = params.find(name);
  return it == params.end() ? def : it->second;
}

const avatar_archetype *world_rules::find_avatar(const std::string &id) const {
  for (const avatar_archetype &a : archetypes)
    if (a.id == id)
      return &a;
  return nullptr;
}

const avatar_archetype &world_rules::avatar() const {
  for (const avatar_archetype &a : archetypes) // the first person: a vehicle has no body here
    if (a.vehicle.empty())
      return a;
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
std::vector<std::string> boat_presets() { return {"launch", "jonboat", "tactical"}; }
bool vehicle_plant_floats(const std::string &plant) { return plant == "dynamicboat"; }

std::string vehicle_controls(const vehicle_archetype &a) {
  if (!a.controls.empty())
    return a.controls;
  return a.plant.rfind("dynamicboat", 0) == 0 ? "boat" : "car";
}

bool vehicle_controller_known(const std::string &id) {
  return id == "nav@1" || id == "none" || id == "external" || id == "pose";
}

bool spawn_policy_known(const std::string &policy) {
  return policy == "fixed" || policy == "random" || policy == "round_robin" ||
         policy == "nearest_free";
}

bool spawn_scope_known(const std::string &scope) {
  return scope == "team" || scope == "archetype" || scope == "all";
}

bool vehicle_plant_known(const std::string &plant) {
  return plant == "jolt" || plant == "kinematic" || plant == "dynamic" || plant == "dynamic4w" ||
         plant == "dynamicboat";
}

std::string fleet_plants::of(int member, int team) const {
  if (const auto it = teams.find(team); team >= 0 && it != teams.end())
    return it->second;
  return cycle.empty() ? std::string()
                       : cycle[static_cast<std::size_t>(std::max(0, member)) % cycle.size()];
}

std::string fleet_plants::text() const {
  std::string out;
  for (const std::string &p : cycle)
    out += (out.empty() ? "" : ", ") + p;
  for (const auto &[t, p] : teams)
    out += (out.empty() ? "" : ", ") + std::to_string(t) + ": " + p;
  return out;
}

std::string parse_fleet_plants(const std::string &text, fleet_plants &out, int teams) {
  fleet_plants p;
  const auto trim = [](std::string s) {
    const std::size_t a = s.find_first_not_of(" \t"), b = s.find_last_not_of(" \t");
    return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
  };
  std::size_t at = 0;
  while (at <= text.size()) {
    const std::size_t comma = std::min(text.find(',', at), text.size());
    const std::string item = trim(text.substr(at, comma - at));
    at = comma + 1;
    if (item.empty())
      continue;
    const std::size_t colon = item.find(':');
    const std::string plant = trim(colon == std::string::npos ? item : item.substr(colon + 1));
    if (!vehicle_plant_known(plant))
      return "'" + plant + "' is not a plant (jolt, kinematic, dynamic, dynamic4w, dynamicboat)";
    if (colon == std::string::npos) {
      p.cycle.push_back(plant);
      continue;
    }
    const std::string ts = trim(item.substr(0, colon));
    char *end = nullptr;
    const long t = std::strtol(ts.c_str(), &end, 10);
    if (ts.empty() || !end || *end || t < 0 || t >= kMaxFleetRally)
      return "'" + ts + "' is not a team (0 to " + std::to_string(kMaxFleetRally - 1) + ")";
    if (teams >= 0 && t >= teams)
      return "team " + ts + " is not one of the fleet's " +
             (teams == 0 ? std::string("(it has no rally points)")
                         : "0 to " + std::to_string(teams - 1));
    if (!p.teams.emplace(static_cast<int>(t), plant).second)
      return "team " + ts + " is named twice";
  }
  if (p.empty())
    return "names no plant";
  std::size_t floating = 0;
  for (const std::string &q : p.cycle)
    floating += vehicle_plant_floats(q) ? 1 : 0;
  for (const auto &[t, q] : p.teams)
    floating += vehicle_plant_floats(q) ? 1 : 0;
  if (floating > 0 && floating < p.cycle.size() + p.teams.size())
    return "its plants all float (dynamicboat) or none do";
  out = std::move(p);
  return std::string();
}

bool fleet_metric_known(const std::string &metric) {
  return metric == "trained" || metric == "vehicle" || metric == "policy";
}

world_rules default_world_rules() { return world_rules{}; }

bool controller_kernel_known(const std::string &kernel, int version) {
  return kernel == "walker" && version == 1;
}

std::string check_world_rules(const world_rules &r) {
  const cvc::physics::vec3 &g = r.physics.gravity;
  if (!std::isfinite(g.x) || !std::isfinite(g.y) || !std::isfinite(g.z))
    return "physics: gravity must be finite";
  const world_physics &wp = r.physics;
  if (!std::isfinite(wp.wind_speed) || !std::isfinite(wp.wind_dir) ||
      !std::isfinite(wp.wind_gustiness) || !std::isfinite(wp.wind_base) || wp.wind_speed < 0.0 ||
      wp.wind_gustiness < 0.0)
    return "physics: the wind must be finite, its speed and gustiness >= 0";

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
    if (!a.vehicle.empty()) { // its body is the vehicle's: checked against vehicles: below
      bool offered = false;
      for (const vehicle_archetype &v : r.vehicles)
        offered = offered || v.id == a.vehicle;
      if (!offered)
        return at + " is vehicle '" + a.vehicle + "', which vehicles: archetypes: does not define";
      continue;
    }
    if (!finite_positive(a.body.mass) || !finite_positive(a.body.width) ||
        !finite_positive(a.body.height) || !finite_positive(a.body.density))
      return at + ": body mass, width, height and density must be positive";
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
      return at + ": plant '" + v.plant +
             "' is not one of jolt, kinematic, dynamic, dynamic4w, dynamicboat";
    const bool boat = vehicle_plant_floats(v.plant);
    bool preset = false;
    for (const std::string &p : boat ? boat_presets() : vehicle_presets())
      preset = preset || p == v.preset;
    if (!preset)
      return at + ": vehicle '" + v.preset + "' is not a " +
             (boat ? "boat preset (launch, jonboat, tactical)" : "preset (humvee, roboracer)");
    if (boat)
      for (const auto &[k, x] : v.overrides)
        if (k != "mass" && k != "max_speed")
          return at + ": vehicle: " + k + " is not a boat's (mass, max_speed)";
    for (const auto &[k, x] : v.overrides)
      if (k == "max_speed" ? !finite_nonnegative(x) : !finite_positive(x))
        return at + ": vehicle: " + k + " must be " +
               (k == "max_speed" ? "zero or more" : "positive");
    if (!v.look.kind.empty() && v.look.kind != "box" && v.look.path.empty() && !v.look.parted())
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
        return at + " allows controller '" + c + "'; a vehicle has nav@1, none, external or pose";
  }

  if (!spawn_policy_known(r.spawns.policy))
    return "spawns: policy '" + r.spawns.policy +
           "' is not one of fixed, random, round_robin, nearest_free";
  if (!spawn_scope_known(r.spawns.scope))
    return "spawns: scope '" + r.spawns.scope + "' is not one of team, archetype, all";
  if (!finite_nonnegative(r.spawns.ford_depth) || !finite_nonnegative(r.spawns.keel_clearance) ||
      !finite_nonnegative(r.spawns.shore_clearance))
    return "spawns: ford_depth, keel_clearance and shore_clearance must be zero or more";
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
      if (std::string e = parse_vehicles(v, rules.vehicles, rules.vehicle_intents,
                                         rules.vehicle_poses, rules.vehicle_starts, warnings);
          !e.empty())
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

void register_world_block(cvc::app &app) {
  register_ari_block(app, "world", [](const Value &content, LoadResult &out) {
    out.extras.emplace_back("world", content);
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
  if (scene.wind.present) { // the scene's wind pushes the room's avatars too (scene_wind.h)
    r.physics.wind_speed = scene.wind.speed;
    r.physics.wind_dir = scene.wind.dir;
    r.physics.wind_gustiness = scene.wind.gustiness;
    r.physics.wind_base = scene.wind.base;
  }
  if (ph.present) {
    r.physics.gravity = cvc::physics::vec3{ph.gravity[0], ph.gravity[1], ph.gravity[2]};
    // The room's quantum: its rate's (scene: clock: hz, ARIADNE_WORLDS.md section 18.9).
    const double hz = scene.clock.hz > 0.0 ? scene.clock.hz : cvc::kSimRateDefault;
    if (ph.fixed_dt != 0.0 && std::fabs(ph.fixed_dt - cvc::sim_quantum(hz)) > 1e-9)
      warn(warnings, "scene: physics: fixed_dt " + num_text(ph.fixed_dt) +
                         " is ignored: a room steps its avatars every 1/" + num_text(hz) +
                         " s (its rate is scene: clock: hz)");
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
