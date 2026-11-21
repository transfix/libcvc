/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A room's entities -- see inc/cvc/ariadne/conference/room_entities.h.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/conference/room_entities.h>
#include <cvc/ariadne/pose_stream.h>
#include <cvc/ariadne/world_vehicles.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <set>
#include <utility>

namespace cvc {
namespace ariadne {
namespace conference {

namespace {

constexpr const char *kKeyPrefix = "e";
constexpr const char *kPidPrefix = "~e";

std::string triple(const std::array<float, 3> &v) {
  char buf[96];
  std::snprintf(buf, sizeof buf, "%.9g,%.9g,%.9g", v[0], v[1], v[2]);
  return buf;
}
std::string triple(const std::array<double, 3> &v) {
  char buf[96];
  std::snprintf(buf, sizeof buf, "%.17g,%.17g,%.17g", v[0], v[1], v[2]);
  return buf;
}
template <typename T> void read_triple(const std::string &s, std::array<T, 3> &out) {
  const char *p = s.c_str();
  std::array<T, 3> v = out;
  for (int i = 0; i < 3; ++i) {
    char *end = nullptr;
    const double d = std::strtod(p, &end);
    if (end == p || !std::isfinite(d))
      return; // malformed: keep the default
    v[i] = static_cast<T>(d);
    p = end;
    if (i < 2) {
      if (*p != ',')
        return;
      ++p;
    }
  }
  out = v;
}

// The ids under the section's subtree, from their keys (e<id>).
std::vector<std::uint64_t> section_ids(cvc::app &app, const std::string &base) {
  std::vector<std::uint64_t> out;
  cvc::state *n = app.root().findDescendant(base);
  if (!n)
    return out;
  const std::string prefix = base + cvc::state::SEPARATOR;
  for (const std::string &full : n->children()) {
    if (full.rfind(prefix, 0) != 0)
      continue;
    const std::string rel = full.substr(prefix.size());
    if (rel.size() < 2 || rel.find(cvc::state::SEPARATOR) != std::string::npos ||
        rel.rfind(kKeyPrefix, 0) != 0)
      continue;
    const std::string digits = rel.substr(1);
    if (!std::all_of(digits.begin(), digits.end(), [](char c) { return c >= '0' && c <= '9'; }))
      continue;
    out.push_back(std::strtoull(digits.c_str(), nullptr, 10));
  }
  std::sort(out.begin(), out.end());
  return out;
}

} // namespace

std::string entity_key(std::uint64_t id) { return kKeyPrefix + std::to_string(id); }
std::string entity_pid(std::uint64_t id) { return kPidPrefix + std::to_string(id); }

bool parse_entity_pid(const std::string &pid, std::uint64_t &id) {
  const std::size_t n = std::char_traits<char>::length(kPidPrefix);
  if (pid.size() <= n || pid.compare(0, n, kPidPrefix) != 0)
    return false;
  const std::string digits = pid.substr(n);
  if (!std::all_of(digits.begin(), digits.end(), [](char c) { return c >= '0' && c <= '9'; }))
    return false;
  id = std::strtoull(digits.c_str(), nullptr, 10);
  return true;
}

bool is_entity_pid(const std::string &pid) {
  std::uint64_t id = 0;
  return parse_entity_pid(pid, id);
}

std::string room_entities_path(const std::string &room) {
  const std::string sep(cvc::state::SEPARATOR);
  return "conf" + sep + "rooms" + sep + room + sep + "entities";
}

section_stamp write_room_entities(cvc::app &app, const std::string &room,
                                  const std::vector<room_entity> &entities, std::uint64_t epoch) {
  const std::string base = room_entities_path(room);
  section_writer w(app, base);
  std::set<std::uint64_t> written;
  for (const room_entity &e : entities) {
    const std::string k = entity_key(e.id);
    w.set({k, "name"}, e.name);
    w.set({k, "archetype"}, e.archetype);
    w.set({k, "plant"}, e.plant);
    w.set({k, "controller"}, e.controller);
    w.set({k, "driver"}, e.driver);
    w.set({k, "team"}, std::to_string(e.team));
    w.set({k, "color"}, triple(e.color));
    w.set({k, "color2"}, triple(e.color2));
    w.set({k, "size"}, triple(e.size));
    w.set({k, "wheels"}, triple(e.wheels));
    w.set_double({k, "base_z"}, e.base_z);
    w.set_double({k, "fit_length"}, e.fit_length);
    w.set({k, "look", "kind"}, e.look.kind);
    w.set({k, "look", "path"}, e.look.path);
    w.set_double({k, "look", "scale"}, e.look.scale);
    w.set_double({k, "look", "yaw"}, e.look.yaw);
    w.set_double({k, "look", "z"}, e.look.z);
    w.set_flag({k, "look", "fit"}, e.look.fit);
    w.set_flag({k, "drivable"}, e.drivable);
    w.set_double({k, "reach"}, e.reach);
    w.set({k, "controls"}, e.controls);
    w.set({k, "avatar"}, e.avatar);
    w.set_flag({k, "present"}, true); // last: a reader that sees it present sees its fields
    written.insert(e.id);
  }
  const section_reader r(app, base);
  for (std::uint64_t id : section_ids(app, base))
    if (!written.count(id) && r.flag({entity_key(id), "present"}, false))
      w.set_flag({entity_key(id), "present"}, false);
  return w.commit(0, epoch);
}

std::vector<room_entity> read_room_entities(cvc::app &app, const std::string &room) {
  const std::string base = room_entities_path(room);
  const section_reader r(app, base);
  std::vector<room_entity> out;
  for (std::uint64_t id : section_ids(app, base)) {
    const std::string k = entity_key(id);
    if (!r.flag({k, "present"}, false))
      continue;
    room_entity e;
    e.id = id;
    e.name = r.text({k, "name"});
    e.archetype = r.text({k, "archetype"});
    e.plant = r.text({k, "plant"});
    e.controller = r.text({k, "controller"});
    e.driver = r.text({k, "driver"});
    e.team = static_cast<int>(r.number({k, "team"}, 0.0));
    read_triple(r.text({k, "color"}), e.color);
    read_triple(r.text({k, "color2"}), e.color2);
    read_triple(r.text({k, "size"}), e.size);
    read_triple(r.text({k, "wheels"}), e.wheels);
    e.base_z = r.number({k, "base_z"}, -1.0);
    e.fit_length = r.number({k, "fit_length"}, 0.0);
    e.look.kind = r.text({k, "look", "kind"});
    e.look.path = r.text({k, "look", "path"});
    e.look.scale = r.number({k, "look", "scale"}, 1.0);
    e.look.yaw = r.number({k, "look", "yaw"}, 0.0);
    e.look.z = r.number({k, "look", "z"}, 0.0);
    e.look.fit = r.flag({k, "look", "fit"}, false);
    e.drivable = r.flag({k, "drivable"}, false);
    e.reach = r.number({k, "reach"}, 3.0);
    e.controls = r.text({k, "controls"});
    e.avatar = r.text({k, "avatar"});
    out.push_back(std::move(e));
  }
  return out;
}

section_stamp read_room_entities_stamp(cvc::app &app, const std::string &room) {
  return read_section_stamp(app, room_entities_path(room));
}

std::vector<room_entity> room_entities_of(const WorldVehicles &vehicles) {
  std::vector<room_entity> out;
  for (const entity_info &info : vehicles.entities()) {
    if (!info.placed)
      continue;
    room_entity e;
    e.id = info.id;
    e.name = info.spec.name;
    e.archetype = info.spec.archetype;
    e.plant = info.plant;
    e.controller = info.spec.controller;
    e.driver = info.spec.driver;
    e.team = info.spec.team;
    e.color = info.spec.color;
    e.color2 = info.spec.color2;
    e.base_z = info.ground_z - info.z;
    if (const auto av = info.spec.metadata.find("avatar"); av != info.spec.metadata.end())
      e.avatar = av->second;
    for (const vehicle_archetype &a : vehicles.archetypes()) {
      if (a.id != info.spec.archetype)
        continue;
      e.look = a.look;
      e.drivable = a.drivable;
      e.reach = a.reach;
      e.controls = vehicle_controls(a);
      if (e.look.kind == "file" || e.look.kind == "asset")
        e.look.path = std::filesystem::path(e.look.path).filename().string();
      try {
        const physics::vehicle_desc d = vehicle_desc_of(a, info.spec.mass, info.spec.max_speed);
        e.size = {2.0 * d.half_extents.x, 2.0 * d.half_extents.y, 2.0 * d.half_extents.z};
        e.wheels = {d.wheelbase(), d.track(), d.axles.empty() ? 0.0 : d.axles.front().wheel_radius};
        if (vehicle_plant_floats(info.plant)) { // a boat's model spans its overall length
          const physics::dyn_boat_desc b = boat_desc_of(a, info.spec.mass, info.spec.max_speed);
          if (b.length_overall > 0.0)
            e.fit_length = b.length_overall;
        }
      } catch (const std::exception &) {
      }
      break;
    }
    out.push_back(std::move(e));
  }
  std::sort(out.begin(), out.end(),
            [](const room_entity &a, const room_entity &b) { return a.id < b.id; });
  return out;
}

std::vector<pose_subject> entity_pose_subjects(const WorldVehicles &vehicles) {
  std::vector<pose_subject> out;
  for (const entity_info &info : vehicles.entities()) {
    if (!info.placed)
      continue;
    pose_subject s;
    s.pid = entity_pid(info.id);
    s.state.position = {info.x, info.y, info.z};
    s.state.rotation = {std::cos(0.5 * info.yaw), 0.0, 0.0, std::sin(0.5 * info.yaw)};
    s.state.linear_velocity = {info.vx, info.vy, 0.0};
    s.state.active = true;
    out.push_back(std::move(s));
  }
  return out;
}

const std::vector<pose_subject> &room_entity_relay::step(cvc::app &app, const std::string &room,
                                                         const WorldVehicles *vehicles) {
  _subjects.clear();
  if (!vehicles) {
    if (_written || _vehicles) { // the world (or its vehicles) went: the section empties once
      write_room_entities(app, room, {});
      ++_writes;
      _written = false;
      _vehicles = nullptr;
    }
    return _subjects;
  }
  if (vehicles != _vehicles || !_written || vehicles->revision() != _rev) {
    write_room_entities(app, room, room_entities_of(*vehicles));
    ++_writes;
    _vehicles = vehicles;
    _rev = vehicles->revision();
    _written = true;
  }
  _subjects = entity_pose_subjects(*vehicles);
  return _subjects;
}

} // namespace conference
} // namespace ariadne
} // namespace cvc
