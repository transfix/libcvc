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
#include <cvc/ariadne/conference/room_avatar.h> // room_avatars_path
#include <cvc/ariadne/conference/room_rules.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h> // SEPARATOR
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {
namespace conference {

namespace {

std::string room_path(const std::string &room) {
  std::string s = "conf";
  for (const std::string &seg : {std::string("rooms"), room}) {
    s += cvc::state::SEPARATOR;
    s += seg;
  }
  return s;
}

std::string under(const std::string &base, const std::string &seg) {
  return base + cvc::state::SEPARATOR + seg;
}

std::string idx(std::size_t i) { return std::to_string(i); }

// A reader's count, bounded: a list longer than this is a corrupt or hostile tree, not a world.
constexpr std::uint64_t kMaxEntries = 256;

} // namespace

std::string room_physics_path(const std::string &room_id) {
  return under(room_path(room_id), "physics");
}
std::string avatar_archetypes_path(const std::string &room_id) {
  return under(room_avatars_path(room_id), "archetypes");
}
std::string avatar_policy_path(const std::string &room_id) {
  return under(room_avatars_path(room_id), "policy");
}
std::string room_controllers_path(const std::string &room_id) {
  return under(room_path(room_id), "controllers");
}

// ---- physics

section_stamp write_room_physics(cvc::app &app, const std::string &room_id, const world_physics &p,
                                 std::uint64_t at, std::uint64_t epoch) {
  section_writer w(app, room_physics_path(room_id));
  w.set_double({"gravity", "x"}, p.gravity.x);
  w.set_double({"gravity", "y"}, p.gravity.y);
  w.set_double({"gravity", "z"}, p.gravity.z);
  return w.commit(at, epoch);
}

section_stamp room_physics_stamp(cvc::app &app, const std::string &room_id) {
  return read_section_stamp(app, room_physics_path(room_id));
}

bool read_room_physics(cvc::app &app, const std::string &room_id, world_physics &out) {
  if (!room_physics_stamp(app, room_id).written())
    return false;
  const section_reader r(app, room_physics_path(room_id));
  world_physics p;
  p.gravity.x = r.number({"gravity", "x"}, p.gravity.x);
  p.gravity.y = r.number({"gravity", "y"}, p.gravity.y);
  p.gravity.z = r.number({"gravity", "z"}, p.gravity.z);
  out = p;
  return true;
}

// ---- archetypes

section_stamp write_avatar_archetypes(cvc::app &app, const std::string &room_id,
                                      const std::vector<avatar_archetype> &list, std::uint64_t at,
                                      std::uint64_t epoch) {
  section_writer w(app, avatar_archetypes_path(room_id));
  for (std::size_t i = 0; i < list.size(); ++i) {
    const avatar_archetype &a = list[i];
    w.set({idx(i), "id"}, a.id);
    w.set_double({idx(i), "body", "mass"}, a.body.mass);
    w.set_double({idx(i), "body", "width"}, a.body.width);
    w.set_double({idx(i), "body", "height"}, a.body.height);
    for (std::size_t j = 0; j < a.controllers.size(); ++j)
      w.set({idx(i), "controllers", idx(j)}, a.controllers[j]);
    w.set_uint({idx(i), "controllers", "count"}, a.controllers.size());
  }
  w.set_uint({"count"}, list.size());
  return w.commit(at, epoch);
}

section_stamp avatar_archetypes_stamp(cvc::app &app, const std::string &room_id) {
  return read_section_stamp(app, avatar_archetypes_path(room_id));
}

bool read_avatar_archetypes(cvc::app &app, const std::string &room_id,
                            std::vector<avatar_archetype> &out) {
  if (!avatar_archetypes_stamp(app, room_id).written())
    return false;
  const section_reader r(app, avatar_archetypes_path(room_id));
  std::vector<avatar_archetype> list;
  const std::uint64_t n = std::min(r.uint({"count"}, 0), kMaxEntries);
  for (std::uint64_t i = 0; i < n; ++i) {
    const std::string k = idx(i);
    avatar_archetype a;
    a.id = r.text({k, "id"}, a.id);
    a.body.mass = r.number({k, "body", "mass"}, a.body.mass);
    a.body.width = r.number({k, "body", "width"}, a.body.width);
    a.body.height = r.number({k, "body", "height"}, a.body.height);
    a.controllers.clear();
    const std::uint64_t m = std::min(r.uint({k, "controllers", "count"}, 0), kMaxEntries);
    for (std::uint64_t j = 0; j < m; ++j)
      a.controllers.push_back(r.text({k, "controllers", idx(j)}));
    list.push_back(std::move(a));
  }
  out = std::move(list);
  return true;
}

// ---- policy

section_stamp write_avatar_policy(cvc::app &app, const std::string &room_id, const avatar_policy &p,
                                  std::uint64_t at, std::uint64_t epoch) {
  section_writer w(app, avatar_policy_path(room_id));
  w.set_double({"max_speed"}, p.max_speed);
  w.set_flag({"allow_vertical"}, p.allow_vertical);
  const avatar_energy_policy &e = p.energy;
  w.set_flag({"energy", "enabled"}, e.enabled);
  w.set_double({"energy", "force_max"}, e.force_max);
  w.set_double({"energy", "power_max"}, e.power_max);
  w.set_double({"energy", "power_rest"}, e.power_rest);
  w.set_double({"energy", "reserve_max"}, e.reserve_max);
  w.set_double({"energy", "transport_cost"}, e.transport_cost);
  w.set({"energy", "empty"}, e.empty == avatar_energy_policy::when_empty::stop ? "stop" : "rest");
  w.set_double({"energy", "recover_fraction"}, e.recover_fraction);
  return w.commit(at, epoch);
}

section_stamp avatar_policy_stamp(cvc::app &app, const std::string &room_id) {
  return read_section_stamp(app, avatar_policy_path(room_id));
}

bool read_avatar_policy(cvc::app &app, const std::string &room_id, avatar_policy &out) {
  if (!avatar_policy_stamp(app, room_id).written())
    return false;
  const section_reader r(app, avatar_policy_path(room_id));
  avatar_policy p;
  p.max_speed = r.number({"max_speed"}, p.max_speed);
  p.allow_vertical = r.flag({"allow_vertical"}, p.allow_vertical);
  avatar_energy_policy &e = p.energy;
  e.enabled = r.flag({"energy", "enabled"}, e.enabled);
  e.force_max = r.number({"energy", "force_max"}, e.force_max);
  e.power_max = r.number({"energy", "power_max"}, e.power_max);
  e.power_rest = r.number({"energy", "power_rest"}, e.power_rest);
  e.reserve_max = r.number({"energy", "reserve_max"}, e.reserve_max);
  e.transport_cost = r.number({"energy", "transport_cost"}, e.transport_cost);
  e.empty = r.text({"energy", "empty"}) == "stop" ? avatar_energy_policy::when_empty::stop
                                                  : avatar_energy_policy::when_empty::rest;
  e.recover_fraction = r.number({"energy", "recover_fraction"}, e.recover_fraction);
  out = p;
  return true;
}

// ---- controllers

section_stamp write_room_controllers(cvc::app &app, const std::string &room_id,
                                     const std::vector<avatar_controller> &list, std::uint64_t at,
                                     std::uint64_t epoch) {
  section_writer w(app, room_controllers_path(room_id));
  for (std::size_t i = 0; i < list.size(); ++i) {
    const avatar_controller &c = list[i];
    w.set({idx(i), "id"}, c.id);
    w.set({idx(i), "kernel"}, c.kernel);
    w.set_uint({idx(i), "version"}, static_cast<std::uint64_t>(c.version));
    std::string names;
    for (const auto &[name, v] : c.params) {
      w.set_double({idx(i), "params", name}, v);
      names += (names.empty() ? "" : ",") + name;
    }
    w.set({idx(i), "params", "names"}, names); // after the values: a reader takes only these
  }
  w.set_uint({"count"}, list.size());
  return w.commit(at, epoch);
}

section_stamp room_controllers_stamp(cvc::app &app, const std::string &room_id) {
  return read_section_stamp(app, room_controllers_path(room_id));
}

bool read_room_controllers(cvc::app &app, const std::string &room_id,
                           std::vector<avatar_controller> &out) {
  if (!room_controllers_stamp(app, room_id).written())
    return false;
  const section_reader r(app, room_controllers_path(room_id));
  std::vector<avatar_controller> list;
  const std::uint64_t n = std::min(r.uint({"count"}, 0), kMaxEntries);
  for (std::uint64_t i = 0; i < n; ++i) {
    const std::string k = idx(i);
    avatar_controller c;
    c.id = r.text({k, "id"});
    c.kernel = r.text({k, "kernel"});
    c.version = static_cast<int>(r.uint({k, "version"}, 0));
    c.params.clear();
    const std::string names = r.text({k, "params", "names"});
    for (std::size_t at = 0; at < names.size();) {
      const std::size_t comma = names.find(',', at);
      const std::string name = names.substr(at, comma == std::string::npos ? comma : comma - at);
      if (!name.empty() && r.has({k, "params", name}))
        c.params[name] = r.number({k, "params", name}, 0.0);
      if (comma == std::string::npos)
        break;
      at = comma + 1;
    }
    list.push_back(std::move(c));
  }
  out = std::move(list);
  return true;
}

// ---- all four

void write_room_rules(cvc::app &app, const std::string &room_id, const world_rules &r,
                      std::uint64_t at, std::uint64_t epoch) {
  write_room_physics(app, room_id, r.physics, at, epoch);
  write_room_controllers(app, room_id, r.controllers, at, epoch);
  write_avatar_archetypes(app, room_id, r.archetypes, at, epoch);
  write_avatar_policy(app, room_id, r.policy, at, epoch);
}

world_rules read_room_rules(cvc::app &app, const std::string &room_id) {
  world_rules r = default_world_rules();
  read_room_physics(app, room_id, r.physics);
  read_avatar_archetypes(app, room_id, r.archetypes);
  read_avatar_policy(app, room_id, r.policy);
  read_room_controllers(app, room_id, r.controllers);
  return r;
}

} // namespace conference
} // namespace ariadne
} // namespace cvc
