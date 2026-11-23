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
#include <cvc/ariadne/conference/room_avatar.h>     // each reserve
#include <cvc/ariadne/conference/room_entities.h>   // entity pids, the entities section
#include <cvc/ariadne/conference/room_intrinsics.h> // room_participant_path, room_roster
#include <cvc/ariadne/conference/room_rules.h>      // the world rules sections
#include <cvc/ariadne/conference/room_scene.h>      // read_scene_objects, scene_stamp
#include <cvc/ariadne/holon.h>                      // cvc::ariadne::holon (server_pump)
#include <cvc/ariadne/holon_sync.h>
#include <cvc/ariadne/loader.h>         // load_file: the world document
#include <cvc/ariadne/scene_world.h>    // world_replica's world
#include <cvc/ariadne/uri.h>            // resolve
#include <cvc/ariadne/world_vehicles.h> // vehicle_ground_height
#include <cvc/core/app.h>
#include <cvc/core/sim_tick.h> // the room's rate (on_room_clock)
#include <cvc/physics/types.h> // vec3, quat
#include <cvc/physics/world.h> // the replica world's water (snapshot)
#include <cvc/state/state.h>   // cvc::state, SEPARATOR, findDescendant
#include <initializer_list>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

namespace {

// Round-trip-safe double text (17 significant digits exactly recovers a binary64),
// portable across every CI toolchain (no std::to_chars floating-point dependency).
std::string fmt(double d) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.17g", d);
  return std::string(buf);
}

// A leaf path <participant>.<segs...>, joined with cvc::state::SEPARATOR (never an
// embedded literal separator, so this holds whatever SEPARATOR is).
std::string leaf(const std::string &room, const std::string &pid,
                 std::initializer_list<const char *> segs) {
  std::string s = conference::room_participant_path(room, pid);
  for (const char *seg : segs) {
    s += cvc::state::SEPARATOR;
    s += seg;
  }
  return s;
}

// touch-before-value, so a just-created node's first write is not lost to the
// distributed-state adapter's attach race (the room tree's set_replicated idiom). Every call
// re-sends the leaf, changed or not, which is what an intent wants: proposing it again reaches a
// relay whose copy has diverged. Poses use set_pose_leaf instead.
void set_replicated(cvc::app &app, const std::string &path, const std::string &val) {
  cvc::state &n = app.root()(path);
  n.touch();
  n.value(val);
}

double read_double(cvc::app &app, const std::string &path) {
  cvc::state *n = app.root().findDescendant(path);
  return n ? std::strtod(n->value().c_str(), nullptr) : 0.0;
}
bool node_present(cvc::app &app, const std::string &path) {
  return app.root().findDescendant(path) != nullptr;
}
bool leaf_valued(cvc::app &app, const std::string &path) {
  cvc::state *n = app.root().findDescendant(path);
  return n && !n->value().empty();
}

// A pose leaf write that journals ONE mutation, where set_replicated journals two for a leaf whose
// value changes (touch fires valueChanged, then value() fires it again), doubling pose egress,
// backfill and journal growth. A leaf that is new, or exists but has never held a value, may not
// be observed by the distributed-state adapter yet (creating a node signals nothing; the adapter
// attaches it on the childChanged a touch emits), so it keeps touch-before-value and its value is
// the one write that journals. A leaf with a value is observed: a new value alone journals. FORCE
// re-sends an unchanged leaf with a touch (one mutation carrying the current value, for a peer
// that missed it); without it an unchanged leaf costs nothing.
void set_pose_leaf(cvc::app &app, const std::string &path, const std::string &val, bool force) {
  cvc::state *n = app.root().findDescendant(path);
  const std::string cur = n ? n->value() : std::string();
  if (cur.empty()) {
    cvc::state &m = n ? *n : app.root()(path);
    m.touch();
    m.value(val);
  } else if (cur != val) {
    n->value(val);
  } else if (force) {
    n->touch();
  }
}

// set_pose_leaf unless the leaf already holds V to within kPoseEpsilon; true when it wrote. A
// resting rigid body jitters in its last float bits every step; that is not news worth a write.
constexpr double kPoseEpsilon = 1e-4; // m, m/s, quaternion units
bool set_if_changed(cvc::app &app, const std::string &path, double v) {
  if (cvc::state *n = app.root().findDescendant(path);
      n && !n->value().empty() &&
      std::fabs(std::strtod(n->value().c_str(), nullptr) - v) <= kPoseEpsilon)
    return false;
  set_pose_leaf(app, path, fmt(v), /*force=*/false);
  return true;
}

} // namespace

void write_pose(cvc::app &app, const std::string &room, const std::string &pid,
                const cvc::physics::body_state &s, std::uint64_t server_tick,
                std::uint64_t heartbeat_ticks) {
  // Heartbeat 0 is the forced rewrite: every leaf goes out (once), changed or not.
  const bool all = heartbeat_ticks == 0;
  bool changed = false;
  const auto put = [&](std::initializer_list<const char *> parts, double v) {
    const std::string path = leaf(room, pid, parts);
    if (all)
      set_pose_leaf(app, path, fmt(v), /*force=*/true), changed = true;
    else
      changed = set_if_changed(app, path, v) || changed;
  };
  put({"pose", "pos", "x"}, s.position.x);
  put({"pose", "pos", "y"}, s.position.y);
  put({"pose", "pos", "z"}, s.position.z);
  put({"pose", "orient", "w"}, s.rotation.w);
  put({"pose", "orient", "x"}, s.rotation.x);
  put({"pose", "orient", "y"}, s.rotation.y);
  put({"pose", "orient", "z"}, s.rotation.z);
  put({"pose", "vel", "lin", "x"}, s.linear_velocity.x);
  put({"pose", "vel", "lin", "y"}, s.linear_velocity.y);
  put({"pose", "vel", "lin", "z"}, s.linear_velocity.z);
  put({"pose", "vel", "ang", "x"}, s.angular_velocity.x);
  put({"pose", "vel", "ang", "y"}, s.angular_velocity.y);
  put({"pose", "vel", "ang", "z"}, s.angular_velocity.z);
  const std::string tick_path = leaf(room, pid, {"pose", "server_tick"});
  if (!changed) {
    // Nothing moved: re-stamp only on the heartbeat (clients re-align their clocks on it).
    cvc::state *t = app.root().findDescendant(tick_path);
    const std::uint64_t last = t ? std::strtoull(t->value().c_str(), nullptr, 10) : 0;
    if (t && server_tick >= last && server_tick - last < heartbeat_ticks)
      return;
  }
  set_pose_leaf(app, tick_path, std::to_string(static_cast<unsigned long long>(server_tick)), all);
}

bool read_pose(cvc::app &app, const std::string &room, const std::string &pid,
               cvc::physics::body_state &out, std::uint64_t &out_server_tick) {
  // A sample is whole once its position, orientation and tick have all arrived: the leaves
  // replicate one by one (live mutations, or a push's entries), so a reader can catch a subtree
  // half applied -- a position with no orientation yet, which is no rotation at all, or no tick.
  const std::string tick_path = leaf(room, pid, {"pose", "server_tick"});
  if (!leaf_valued(app, leaf(room, pid, {"pose", "pos", "x"})) ||
      !leaf_valued(app, leaf(room, pid, {"pose", "orient", "w"})) || !leaf_valued(app, tick_path))
    return false; // the pose subtree has not been (wholly) written/replicated yet
  cvc::physics::body_state s;
  s.position = {read_double(app, leaf(room, pid, {"pose", "pos", "x"})),
                read_double(app, leaf(room, pid, {"pose", "pos", "y"})),
                read_double(app, leaf(room, pid, {"pose", "pos", "z"}))};
  s.rotation = {read_double(app, leaf(room, pid, {"pose", "orient", "w"})),
                read_double(app, leaf(room, pid, {"pose", "orient", "x"})),
                read_double(app, leaf(room, pid, {"pose", "orient", "y"})),
                read_double(app, leaf(room, pid, {"pose", "orient", "z"}))};
  const double qn = std::sqrt(s.rotation.w * s.rotation.w + s.rotation.x * s.rotation.x +
                              s.rotation.y * s.rotation.y + s.rotation.z * s.rotation.z);
  if (!std::isfinite(qn) || qn < 1e-6)
    return false; // malformed, or caught between its orientation leaves
  s.rotation = {s.rotation.w / qn, s.rotation.x / qn, s.rotation.y / qn, s.rotation.z / qn};
  s.linear_velocity = {read_double(app, leaf(room, pid, {"pose", "vel", "lin", "x"})),
                       read_double(app, leaf(room, pid, {"pose", "vel", "lin", "y"})),
                       read_double(app, leaf(room, pid, {"pose", "vel", "lin", "z"}))};
  s.angular_velocity = {read_double(app, leaf(room, pid, {"pose", "vel", "ang", "x"})),
                        read_double(app, leaf(room, pid, {"pose", "vel", "ang", "y"})),
                        read_double(app, leaf(room, pid, {"pose", "vel", "ang", "z"}))};
  s.active = true;
  cvc::state *t = app.root().findDescendant(tick_path);
  out = s;
  out_server_tick =
      t ? static_cast<std::uint64_t>(std::strtoull(t->value().c_str(), nullptr, 10)) : 0;
  return true;
}

void write_intent(cvc::app &app, const std::string &room, const std::string &pid,
                  const avatar_intent &intent, std::uint64_t tick) {
  set_replicated(app, leaf(room, pid, {"intent", "move", "x"}), fmt(intent.move.x));
  set_replicated(app, leaf(room, pid, {"intent", "move", "y"}), fmt(intent.move.y));
  set_replicated(app, leaf(room, pid, {"intent", "move", "z"}), fmt(intent.move.z));
  set_replicated(app, leaf(room, pid, {"intent", "yaw"}), fmt(intent.yaw));
  // The drive axes (a vehicle the participant drives; shares, avatar_intent), "nan" when it has
  // none, and the walker's jump.
  set_replicated(app, leaf(room, pid, {"intent", "steer"}), fmt(intent.steer));
  set_replicated(app, leaf(room, pid, {"intent", "throttle"}), fmt(intent.throttle));
  set_replicated(app, leaf(room, pid, {"intent", "brake"}), fmt(intent.brake));
  set_replicated(app, leaf(room, pid, {"intent", "jump"}), intent.jump ? "1" : "0");
  set_replicated(app, leaf(room, pid, {"intent", "tick"}),
                 std::to_string(static_cast<unsigned long long>(tick)));
}

bool read_intent(cvc::app &app, const std::string &room, const std::string &pid, avatar_intent &out,
                 std::uint64_t *out_tick) {
  if (!node_present(app, leaf(room, pid, {"intent", "move", "x"})))
    return false;
  out.move = {read_double(app, leaf(room, pid, {"intent", "move", "x"})),
              read_double(app, leaf(room, pid, {"intent", "move", "y"})),
              read_double(app, leaf(room, pid, {"intent", "move", "z"}))};
  out.yaw = read_double(app, leaf(room, pid, {"intent", "yaw"}));
  // The drive axes: absent (an older client) is none.
  const auto axis = [&](const char *name) {
    const std::string path = leaf(room, pid, {"intent", name});
    return node_present(app, path) ? read_double(app, path)
                                   : std::numeric_limits<double>::quiet_NaN();
  };
  out.steer = axis("steer");
  out.throttle = axis("throttle");
  out.brake = axis("brake");
  const double jump = axis("jump");
  out.jump = std::isfinite(jump) && jump != 0.0;
  if (out_tick) {
    cvc::state *t = app.root().findDescendant(leaf(room, pid, {"intent", "tick"}));
    *out_tick = t ? std::strtoull(t->value().c_str(), nullptr, 10) : 0;
  }
  return true;
}

std::uint64_t request_drive(cvc::app &app, const std::string &room, const std::string &pid,
                            std::uint64_t entity) {
  const std::string path = leaf(room, pid, {"drive"});
  std::uint64_t seq = read_drive_status(app, room, pid).seq;
  if (cvc::state *n = app.root().findDescendant(path))
    seq = std::max<std::uint64_t>(seq, std::strtoull(n->value().c_str(), nullptr, 10));
  ++seq;
  set_replicated(app, path,
                 std::to_string(static_cast<unsigned long long>(seq)) + ":" +
                     std::to_string(static_cast<unsigned long long>(entity)));
  return seq;
}

std::uint64_t read_driving(cvc::app &app, const std::string &room, const std::string &pid) {
  cvc::state *n = app.root().findDescendant(leaf(room, pid, {"driving"}));
  return n ? std::strtoull(n->value().c_str(), nullptr, 10) : 0;
}

drive_status read_drive_status(cvc::app &app, const std::string &room, const std::string &pid) {
  drive_status out;
  cvc::state *n = app.root().findDescendant(leaf(room, pid, {"drive_status"}));
  if (!n)
    return out;
  const std::string v = n->value();
  char *end = nullptr;
  out.seq = std::strtoull(v.c_str(), &end, 10);
  const std::string rest = end ? std::string(end) : std::string();
  out.ok = rest == " ok";
  const std::string refused = " refused: ";
  if (rest.compare(0, refused.size(), refused) == 0)
    out.why = rest.substr(refused.size());
  return out;
}

void serve_drive(holon &h, cvc::app &app, const std::string &room, const std::string &pid) {
  // A new ask: "<seq>:<entity>", answered once (its seq in drive_status).
  if (cvc::state *ask = app.root().findDescendant(leaf(room, pid, {"drive"}))) {
    const std::string v = ask->value();
    const std::size_t colon = v.find(':');
    char *end = nullptr;
    const std::uint64_t seq = colon == std::string::npos ? 0 : std::strtoull(v.c_str(), &end, 10);
    if (seq != 0 && end == v.c_str() + colon && read_drive_status(app, room, pid).seq != seq) {
      const std::uint64_t entity = std::strtoull(v.c_str() + colon + 1, nullptr, 10);
      std::string why;
      const bool ok = h.get_in(pid, entity, &why);
      set_replicated(app, leaf(room, pid, {"drive_status"}),
                     std::to_string(static_cast<unsigned long long>(seq)) +
                         (ok ? std::string(" ok") : " refused: " + why));
    }
  }
  // What it drives, when that changed (a get-in, a get-out, the vehicle gone); nothing written for
  // one that never drove.
  const std::uint64_t now = h.drives(pid);
  const std::string driving = leaf(room, pid, {"driving"});
  if (node_present(app, driving) ? read_driving(app, room, pid) != now : now != 0)
    set_replicated(app, driving, std::to_string(static_cast<unsigned long long>(now)));
}

void request_archetype(cvc::app &app, const std::string &room, const std::string &pid,
                       const std::string &archetype) {
  set_replicated(app, leaf(room, pid, {"avatar", "archetype"}), archetype);
}

std::string read_requested_archetype(cvc::app &app, const std::string &room,
                                     const std::string &pid) {
  cvc::state *n = app.root().findDescendant(leaf(room, pid, {"avatar", "archetype"}));
  return n ? n->value() : std::string();
}

archetype_status read_archetype_status(cvc::app &app, const std::string &room,
                                       const std::string &pid) {
  // "<archetype>:ok" or "<archetype>:refused: <why>"
  archetype_status out;
  cvc::state *n = app.root().findDescendant(leaf(room, pid, {"archetype_status"}));
  if (!n || n->value().empty())
    return out;
  const std::string v = n->value();
  const std::size_t colon = v.find(':');
  if (colon == std::string::npos)
    return out;
  out.answered = true;
  out.archetype = v.substr(0, colon);
  const std::string rest = v.substr(colon + 1);
  out.ok = rest == "ok";
  const std::string refused = "refused: ";
  if (rest.compare(0, refused.size(), refused) == 0)
    out.why = rest.substr(refused.size());
  return out;
}

void serve_archetype(holon &h, cvc::app &app, const std::string &room, const std::string &pid) {
  const std::string want = read_requested_archetype(app, room, pid);
  const archetype_status st = read_archetype_status(app, room, pid);
  if (st.answered && st.archetype == want)
    return; // ruled on already (a refusal waits for a new ask)
  std::string why;
  const bool ok = h.set_archetype(pid, want, &why);
  // Said when it was asked for, refused, or made a vehicle: a person who never asked writes
  // nothing.
  if (!want.empty() || !ok || h.embodies(pid) != 0)
    set_replicated(app, leaf(room, pid, {"archetype_status"}),
                   want + (ok ? std::string(":ok") : ":refused: " + why));
}

std::uint64_t drive_target(const entity_playout &entities, const cvc::physics::vec3 &feet,
                           double facing, double *out_gap) {
  std::uint64_t best = 0;
  double best_score = 1e300, best_gap = 0.0;
  for (const conference::room_entity &e : entities.entities()) {
    cvc::physics::pose at;
    if (!e.drivable || !e.avatar.empty() || !entities.pose(e.id, at))
      continue; // someone's body is not a ride
    // Its feet to the chassis box, in the vehicle's frame, in plan (holon::get_in's rule).
    const cvc::physics::quat &q = at.rotation;
    const double yaw =
        std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double dx = feet.x - at.position.x, dy = feet.y - at.position.y;
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double lx = c * dx + s * dy, ly = -s * dx + c * dy;
    const double gap = std::hypot(std::max(0.0, std::fabs(lx) - 0.5 * e.size[0]),
                                  std::max(0.0, std::fabs(ly) - 0.5 * e.size[1]));
    if (gap > e.reach)
      continue;
    // Ahead of it first: one behind it scores as if a reach further away.
    const bool ahead = -dx * std::cos(facing) - dy * std::sin(facing) >= 0.0;
    const double score = gap + (ahead ? 0.0 : e.reach);
    if (score < best_score) {
      best = e.id;
      best_score = score;
      best_gap = gap;
    }
  }
  if (out_gap)
    *out_gap = best_gap;
  return best;
}

bool read_current_intent(cvc::app &app, const std::string &room, const std::string &pid,
                         std::uint64_t proposed_tick, avatar_intent &out) {
  avatar_intent tree;
  std::uint64_t stamp = 0;
  if (!read_intent(app, room, pid, tree, &stamp))
    return false;
  if (stamp != 0 && stamp < proposed_tick)
    return false; // an echo of an earlier proposal
  out = tree;
  return true;
}

holon_config room_holon_config() {
  holon_config cfg;
  cfg.physics = true; // the rest is the room's: its rules sections (sync_rules)
  return cfg;
}

holon_config room_holon_config(double hz) {
  return hz > 0.0 ? holon_config_at(room_holon_config(), hz) : room_holon_config();
}

holon_config room_holon_config(cvc::app &app, const std::string &room) {
  return room_holon_config(conference::read_room_clock(app, room));
}

bool on_room_clock(const holon &h, cvc::app &app, const std::string &room) {
  const double hz = conference::read_room_clock(app, room);
  const double quantum = cvc::sim_quantum(hz > 0.0 ? hz : cvc::kSimRateDefault);
  return std::fabs(h.clock().fixed_dt() - quantum) < 1e-12;
}

std::vector<avatar_static> room_statics(cvc::app &app, const std::string &room) {
  constexpr double kDeg = 3.14159265358979323846 / 180.0;
  const auto axis_quat = [](double rad, int axis) {
    cvc::physics::quat q{std::cos(0.5 * rad), 0.0, 0.0, 0.0};
    (axis == 0 ? q.x : axis == 1 ? q.y : q.z) = std::sin(0.5 * rad);
    return q;
  };
  const auto mul = [](const cvc::physics::quat &a, const cvc::physics::quat &b) {
    return cvc::physics::quat{a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
                              a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                              a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                              a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
  };
  std::vector<avatar_static> out;
  for (const cvc::ariadne::SceneNode &n : conference::read_scene_objects(app, room)) {
    avatar_static s;
    s.center = {n.position[0], n.position[1], n.position[2]};
    // X, then Y, then Z: q = qz * qy * qx.
    s.rotation = mul(axis_quat(n.rotation[2] * kDeg, 2),
                     mul(axis_quat(n.rotation[1] * kDeg, 1), axis_quat(n.rotation[0] * kDeg, 0)));
    if (n.source_primitive == "plane") {
      // A slab 0.2 m thick whose top face is the plane.
      constexpr double kSlab = 0.1;
      s.half_extents = {0.5 * n.plane_size * n.scale[0], 0.5 * n.plane_size * n.scale[1], kSlab};
      s.center.z -= kSlab;
    } else if (n.source_primitive == "box") {
      s.half_extents = {0.5 * n.box_size[0] * n.scale[0], 0.5 * n.box_size[1] * n.scale[1],
                        0.5 * n.box_size[2] * n.scale[2]};
    } else if (n.source_primitive == "sphere") {
      s.type = avatar_static::kind::sphere;
      s.radius = n.sphere_radius * std::max({n.scale[0], n.scale[1], n.scale[2]});
    } else {
      continue; // a mesh file: no physical shape yet
    }
    out.push_back(s);
  }
  return out;
}

namespace {
// Physics: reload the statics when the room scene changed (its stamp moved: a new revision, or a
// restarted relay's revision 1 under a new epoch).
void sync_statics(holon &h, cvc::app &app, const std::string &room,
                  world_replica *world = nullptr) {
  if (!h.physics())
    return;
  // The world document's statics, once this client holds the relay's document (OW5c).
  if (world) {
    world->pump(app, room);
    if (world->ground_revision() != h.vehicle_ground_revision())
      h.set_vehicle_ground(world->ground(), world->ground_revision()); // OW5g
    if (world->ready()) {
      if (!world->stamp().same(h.statics_stamp()))
        h.set_statics(world->statics(), world->stamp(), world->water());
      return;
    }
  }
  const section_stamp s = conference::scene_stamp(app, room);
  if (s.written() && !s.same(h.statics_stamp()))
    h.set_statics(room_statics(app, room), s);
}
} // namespace

namespace {

// The room's rules sections (room_rules.h) into H, each whenever its stamp moves: the relay's holon
// and each client's run under the same ones, read from the same leaves, each from its activation
// tick (holon::set_physics). Controllers before the archetypes that name them.
void sync_rules(holon &h, cvc::app &app, const std::string &room) {
  if (const section_stamp s = conference::room_physics_stamp(app, room);
      s.written() && !s.same(h.rules_stamp(rules_section::physics))) {
    world_physics p;
    if (conference::read_room_physics(app, room, p))
      h.set_physics(p, s);
  }
  if (const section_stamp s = conference::room_controllers_stamp(app, room);
      s.written() && !s.same(h.rules_stamp(rules_section::controllers))) {
    std::vector<avatar_controller> c;
    if (conference::read_room_controllers(app, room, c))
      h.set_controllers(c, s);
  }
  if (const section_stamp s = conference::avatar_archetypes_stamp(app, room);
      s.written() && !s.same(h.rules_stamp(rules_section::archetypes))) {
    std::vector<avatar_archetype> a;
    if (conference::read_avatar_archetypes(app, room, a))
      h.set_archetypes(a, s);
  }
  if (const section_stamp s = conference::avatar_policy_stamp(app, room);
      s.written() && !s.same(h.rules_stamp(rules_section::policy))) {
    avatar_policy p;
    if (conference::read_avatar_policy(app, room, p))
      h.set_policy(p, s);
  }
}

// Publish PID's reserve when it has moved by half a percent of the most it can hold, when it
// first empties or fills, and otherwise every HEARTBEAT ticks while it still differs from the
// published one. A reserve that has stopped moving (full, at rest) needs no re-stamp: the client's
// prediction refills to the same value, and it was corrected on the way there.
void write_reserve(holon &h, cvc::app &app, const std::string &room, const std::string &pid,
                   std::uint64_t heartbeat) {
  avatar_energy e;
  if (!h.energy(pid, e))
    return;
  double last = 0.0;
  std::uint64_t last_tick = 0;
  const bool have = conference::read_avatar_reserve(app, room, pid, last, last_tick);
  const double full = h.energy_policy().reserve_max;
  const double now = std::floor(e.reserve + 0.5); // published in whole joules
  const bool moved = !have || std::fabs(now - last) >= 0.005 * full ||
                     ((now == 0.0 || now == std::floor(full + 0.5)) && now != last);
  const bool beat = h.tick() < last_tick || (now != last && h.tick() - last_tick >= heartbeat);
  if (moved || beat)
    conference::write_avatar_reserve(app, room, pid, e.reserve, h.tick());
}

} // namespace

void server_pump(holon &h, cvc::app &app, const std::string &room, double wall_dt, bool write_poses,
                 double pose_heartbeat_s) {
  sync_statics(h, app, room);
  sync_rules(h, app, room);
  // The present participants, straight from the replicated room tree.
  const std::vector<conference::room_participant> roster = conference::room_roster(app, room);
  // Add any newcomer -- spawned on a 4 m ring around the room's origin (where the seeded
  // reference object stands), each at the next golden-angle step so joiners never stack
  // -- then feed each one's latest proposed intent to the holon.
  for (const conference::room_participant &p : roster) {
    if (!h.has_participant(p.pid)) {
      constexpr double kSpawnRadius = 4.0;
      constexpr double kGoldenAngle = 2.39996322972865332; // radians
      const double a = kGoldenAngle * static_cast<double>(h.participants().size());
      cvc::physics::body_state start;
      start.position = {kSpawnRadius * std::cos(a), kSpawnRadius * std::sin(a), 0.0};
      h.add_participant(p.pid, /*local=*/false, start);
    }
    avatar_intent in;
    std::uint64_t stamp = 0;
    if (read_intent(app, room, p.pid, in, &stamp)) {
      if (stamp != 0)
        h.queue_intent(p.pid, in, stamp); // applied at its tick (no-op when already queued)
      else
        h.set_intent(p.pid, in); // untimed (a DSL verb, a departure's reset): at once
    }
  }
  // Drop anyone who left (room_leave): the holon stops integrating them, and a later rejoin under
  // the same id respawns them on the ring rather than wherever a stale intent carried them.
  std::set<std::string> present;
  for (const conference::room_participant &p : roster)
    present.insert(p.pid);
  for (const std::string &pid : h.participants())
    if (!present.count(pid))
      h.remove_participant(pid);
  // Each one's archetype (a vehicle is its body), then getting in and out of the world's vehicles
  // (section 18.6).
  for (const conference::room_participant &p : roster) {
    serve_archetype(h, app, room, p.pid);
    serve_drive(h, app, room, p.pid);
  }
  // One authoritative frame.
  h.advance(wall_dt);
  // Publish each participant's authoritative pose (these are local writes on the
  // relay's shard; the caller fans them to the clients): what changed, and a
  // heartbeat of POSE_HEARTBEAT_S for an avatar standing still -- unless the caller
  // streams the poses this step (WRITE_POSES false).
  const std::uint64_t heartbeat =
      std::max<std::uint64_t>(1, static_cast<std::uint64_t>(1.0 / h.clock().fixed_dt()));
  const auto pose_heartbeat = std::max<std::uint64_t>(
      1, static_cast<std::uint64_t>(std::llround(pose_heartbeat_s / h.clock().fixed_dt())));
  if (write_poses)
    for (const conference::room_participant &p : roster)
      write_pose(app, room, p.pid, h.state(p.pid), h.tick(), pose_heartbeat);
  // And each one's reserve, every step: no snapshot carries it, and write_reserve thins it by its
  // own rule, not the poses'.
  for (const conference::room_participant &p : roster)
    write_reserve(h, app, room, p.pid, 5 * heartbeat);
}

namespace {
// client_pump's ENTITIES: each one shown that SELF_PID does not drive or embody, held where it is
// shown; the rest released.
void hold_entities(holon &h, cvc::app &app, const std::string &room, const std::string &self_pid,
                   const entity_playout *entities) {
  std::set<std::uint64_t> keep;
  if (entities && h.physics()) {
    const std::uint64_t driving = read_driving(app, room, self_pid);
    for (const conference::room_entity &e : entities->entities()) {
      if (e.id == driving || e.driver == self_pid || e.avatar == self_pid)
        continue;
      cvc::physics::body_state at;
      if (entities->state(e.id, at) && h.hold_entity(e.id, e.size, at))
        keep.insert(e.id);
    }
  }
  for (std::uint64_t id : h.held_entities())
    if (!keep.count(id))
      h.release_entity(id);
}
} // namespace

void client_pump(holon &h, cvc::app &app, const std::string &room, const std::string &self_pid,
                 double wall_dt, const entity_playout *entities, world_replica *world) {
  sync_statics(h, app, room, world);
  sync_rules(h, app, room);
  const std::vector<conference::room_participant> roster = conference::room_roster(app, room);
  std::set<std::string> present;
  for (const conference::room_participant &p : roster) {
    present.insert(p.pid);
    cvc::physics::body_state s;
    std::uint64_t server_tick = 0;
    bool have = read_pose(app, room, p.pid, s, server_tick);
    // A peer back after leaving (an in-process reconnect keeps its pid) still has the pose subtree
    // it left in our tree, which the relay withholds updates to while it is departed: until a
    // newer sample arrives that is no pose -- registering it from that would show it where it left
    // and seek our clock back to the old tick. "Newer" allows for a restarted relay, whose ticks
    // start again near zero: only a tick within kLeftoverTicks at or below the one it left at is
    // the leftover.
    constexpr std::uint64_t kLeftoverTicks = 1200; // 10 s at the 120 Hz holon quantum
    const std::uint64_t left = h.left_tick(p.pid);
    if (have && left != 0 && server_tick <= left && left - server_tick <= kLeftoverTicks)
      have = false;
    // A participant the roster lists before its pose has replicated is registered pose-less: no
    // body and no marker at the placeholder origin until its first sample (holon::pose_known).
    // Driving, our pose is the vehicle's: predicted when the room's entities say how it runs (the
    // kinematic plant, section 18.5, OW5b; the dynamic plants, OW5d and OW5e), else played out as a
    // peer's; out, the walker again.
    const conference::room_entity *ride = nullptr;
    bool local = p.pid == self_pid;
    if (p.pid == self_pid) {
      const std::uint64_t driving = read_driving(app, room, p.pid);
      ride = driving != 0 && entities ? entities->find(driving) : nullptr;
      if (ride && !ride->drive.valid() && !ride->dynamics.valid())
        ride = nullptr;
      local = driving == 0 || ride;
      if (h.has_participant(p.pid) && (local != h.is_local(p.pid) ||
                                       (local && (ride != nullptr) != h.predicts_vehicle(p.pid)))) {
        const cvc::physics::body_state now = h.state(p.pid);
        h.remove_participant(p.pid);
        h.add_participant(p.pid, local, now, /*pose_known=*/true);
      }
    }
    if (!h.has_participant(p.pid))
      h.add_participant(p.pid, local, have ? s : cvc::physics::body_state{},
                        /*pose_known=*/have);
    if (ride && ride->drive.valid())
      h.predict_vehicle(p.pid, ride->drive); // a no-op while its numbers stay the same
    else if (ride)
      h.predict_vehicle(p.pid, ride->dynamics); // a dynamic plant's (OW5d, OW5e)
    if (p.pid == self_pid) {
      // The server's reserve before the pose: a reconcile trims the log the reserve is checked
      // against.
      double reserve = 0.0;
      std::uint64_t reserve_tick = 0;
      if (conference::read_avatar_reserve(app, room, p.pid, reserve, reserve_tick))
        h.apply_authoritative_reserve(p.pid, reserve_tick, reserve);
    }
    if (have)
      h.apply_authoritative(p.pid, s, server_tick);
  }
  for (const std::string &pid : h.participants())
    if (!present.count(pid))
      h.remove_participant(pid);
  hold_entities(h, app, room, self_pid, entities);
  h.advance(wall_dt);
}

std::size_t apply_pose_snapshot(holon &h, pose_stream_decoder &decoder,
                                const cvc::pose_snapshot &snap, entity_playout *entities) {
  std::vector<pose_sample> samples;
  if (!decoder.decode(snap, samples))
    return 0;
  return apply_pose_samples(h, samples, snap.server_tick, entities);
}

std::size_t apply_pose_samples(holon &h, const std::vector<pose_sample> &samples,
                               std::uint64_t server_tick, entity_playout *entities) {
  std::size_t applied = 0;
  for (const pose_sample &s : samples) {
    if (conference::is_entity_pid(s.pid)) {
      if (entities && entities->apply(s.pid, s.state, server_tick))
        ++applied;
      continue;
    }
    if (h.has_participant(s.pid)) {
      h.apply_authoritative(s.pid, s.state, server_tick);
      ++applied;
    }
  }
  return applied;
}

namespace {
holon_config entity_config(holon_config cfg) {
  cfg.physics = false; // no bodies: an entity is only shown
  return cfg;
}
} // namespace

entity_playout::entity_playout(holon_config cfg)
    : _holon(std::make_unique<holon>(holon_role::client, entity_config(std::move(cfg)))) {}
entity_playout::~entity_playout() = default;

bool entity_playout::apply(const std::string &pid, const cvc::physics::body_state &state,
                           std::uint64_t server_tick) {
  if (!conference::is_entity_pid(pid))
    return false;
  if (!_holon->has_participant(pid)) {
    _holon->add_participant(pid, /*local=*/false, state, /*pose_known=*/true);
    ++_rev;
  }
  _holon->apply_authoritative(pid, state, server_tick);
  return true;
}

void entity_playout::pump(cvc::app &app, const std::string &room, double wall_dt) {
  const section_stamp stamp = conference::read_room_entities_stamp(app, room);
  if (!stamp.same(_stamp)) {
    _stamp = stamp;
    _entities = conference::read_room_entities(app, room);
    ++_rev;
  }
  // An entity the section has dropped leaves the playout; until the section is written at all
  // (it replicates after the first samples, perhaps), every sampled one stays.
  if (_stamp.written()) {
    std::set<std::string> present;
    for (const conference::room_entity &e : _entities)
      present.insert(conference::entity_pid(e.id));
    for (const std::string &pid : _holon->participants())
      if (!present.count(pid)) {
        _holon->remove_participant(pid);
        ++_rev;
      }
  }
  _holon->advance(wall_dt);
}

const conference::room_entity *entity_playout::find(std::uint64_t id) const {
  for (const conference::room_entity &e : _entities)
    if (e.id == id)
      return &e;
  return nullptr;
}

std::vector<std::uint64_t> entity_playout::shown() const {
  std::vector<std::uint64_t> out;
  for (const std::string &pid : _holon->participants()) {
    std::uint64_t id = 0;
    if (conference::parse_entity_pid(pid, id))
      out.push_back(id);
  }
  std::sort(out.begin(), out.end());
  return out;
}

bool entity_playout::pose(std::uint64_t id, cvc::physics::pose &out) const {
  const std::string pid = conference::entity_pid(id);
  if (!_holon->has_participant(pid))
    return false;
  out = id != 0 && id == _predicted ? _predicted_at : _holon->display_pose(pid);
  return true;
}

void entity_playout::show_predicted(std::uint64_t id, const cvc::physics::pose &at) {
  _predicted = id;
  _predicted_at = at;
}

bool entity_playout::state(std::uint64_t id, cvc::physics::body_state &out) const {
  cvc::physics::pose shown;
  if (!pose(id, shown))
    return false;
  out = _holon->state(conference::entity_pid(id));
  out.position = shown.position;
  out.rotation = shown.rotation;
  return true;
}

world_replica::world_replica() {
  // Its own epoch, so its stamps are never the room scene's (holon::set_statics takes either).
  std::random_device rd;
  do
    _stamp.epoch = ((static_cast<std::uint64_t>(rd()) << 21) ^ rd()) & ((1ull << 53) - 1);
  while (_stamp.epoch == 0);
}

world_replica::~world_replica() = default;

conference::room_world_on world_on_of(const SceneWorld &world) {
  conference::room_world_on on;
  on.world = world.world_on();
  // The world on's nodes; a scene without worlds has every node on.
  for (const std::string &node :
       on.world.empty() ? world.generated_nodes() : world.world_nodes(on.world)) {
    conference::room_generated_world g;
    g.node = node;
    if (world.generated_params(node, g.params))
      on.generated.push_back(std::move(g));
  }
  return on;
}

bool publish_world_on(cvc::app &app, const std::string &room, const SceneWorld &world) {
  const conference::room_world_on now = world_on_of(world);
  conference::room_world_on had;
  if (conference::read_room_world_on(app, room, had) && had == now)
    return false;
  conference::write_room_world_on(app, room, now);
  return true;
}

void world_replica::follow(cvc::app &app, const std::string &room) {
  const section_stamp s = conference::room_world_on_stamp(app, room);
  if (s.same(_on_stamp))
    return;
  _on_stamp = s;
  conference::room_world_on on;
  if (!conference::read_room_world_on(app, room, on))
    return;
  std::vector<std::string> warnings;
  if (!on.world.empty() && on.world != _world->world_on())
    _world->set_world(on.world, &warnings);
  for (const conference::room_generated_world &g : on.generated)
    _world->follow_generated(g.node, g.params, &warnings);
}

void world_replica::pump(cvc::app &app, const std::string &room) {
  const section_stamp s = conference::room_world_source_stamp(app, room);
  if (!s.same(_source_stamp)) {
    _source_stamp = s;
    _on_stamp = section_stamp{}; // a new world follows what the relay has on afresh
    _built_on.clear();
    _ground_grid.reset();
    _ground_terrain.reset();
    if (_ground_rev)
      ++_ground_rev; // none now
    _world.reset();
    _nodes.clear();
    _taken.clear();
    _statics.clear();
    _water.clear();
    _ready = false;
    conference::room_world_source src;
    if (!conference::read_room_world_source(app, room, src) || src.uri.empty()) {
      _status = "the room names no world document";
    } else {
      const UriResult found = resolve(app, src.uri);
      if (!found.ok || found.canonical.empty()) {
        _status = src.uri + ": not here" + (found.error.empty() ? "" : " (" + found.error + ")");
      } else if (conference::file_sha256(found.canonical) != src.sha256 || src.sha256.empty()) {
        _status = src.uri + ": not the relay's document (its SHA-256 differs)";
      } else {
        LoadResult r = load_file(app, found.canonical);
        if (!r.ok) {
          _status = src.uri + ": " + r.error;
        } else {
          // Its static world alone: the vehicles are the relay's to run (the room's entities).
          r.scene.vehicles.clear();
          std::vector<std::string> warnings;
          // Its keys (the world switch, a dataset's) under a prefix of its own, apart from the
          // client's document.
          _world = SceneWorld::from_scene(app, r.scene, "world_replica", "", &warnings);
          _status = src.uri + (_world ? ": built" : ": its world could not be made");
          const std::function<void(const std::vector<SceneNode> &)> ids =
              [&](const std::vector<SceneNode> &nodes) {
                for (const SceneNode &n : nodes) {
                  if (!n.id.empty())
                    _nodes.push_back(n.id);
                  ids(n.children);
                }
              };
          ids(r.scene.nodes);
        }
      }
    }
  }
  if (!_world)
    return;
  follow(app, room); // the relay's world on, and its generated worlds' params
  std::vector<std::string> warnings;
  _world->sync(&warnings); // a generated world's colliders, as they become ready
  // Ready once every generated world has its colliders (its terrain is the ground); the statics
  // again whenever the world on or a node's colliders change.
  bool pending = false, changed = !_ready || _world->world_on() != _built_on;
  for (const std::string &id : _nodes) {
    std::uint64_t rev = 0;
    const bool has = _world->world_geometry(id, &rev) != nullptr;
    if (!has && _world->generated_world(id))
      pending = true;
    const auto it = _taken.find(id);
    if (has && (it == _taken.end() || it->second != rev))
      changed = true;
  }
  if (pending && !_ready)
    return;
  if (changed)
    build_statics(app);
  _ready = true;
  refresh_ground();
}

void world_replica::refresh_ground() {
  std::shared_ptr<const cvc::nav::world_grid> g;
  std::shared_ptr<const cvc::height_field> t;
  _world->vehicle_ground(g, t);
  if (g == _ground_grid && t == _ground_terrain)
    return;
  _ground_grid = std::move(g);
  _ground_terrain = std::move(t);
  ++_ground_rev;
}

std::function<double(double, double)> world_replica::ground() const {
  if (!_ground_grid && !_ground_terrain)
    return {};
  std::shared_ptr<const cvc::nav::world_grid> g = _ground_grid;
  std::shared_ptr<const cvc::height_field> t = _ground_terrain;
  return [g, t](double x, double y) { return vehicle_ground_height(g.get(), t.get(), x, y); };
}

void world_replica::build_statics(cvc::app &app) {
  _statics.clear();
  _water.clear();
  _taken.clear();
  _built_on = _world->world_on();
  // What is in its physics world now: a scene's worlds are on one at a time (SceneWorld::world_on),
  // and the others' bodies are out.
  const auto in_world = [this](const SceneWorldBody &b) {
    for (const cvc::physics::body_id id : _world->bodies_of(b.node))
      if (id.value == b.id.value)
        return true;
    return false;
  };
  for (const SceneWorldBody &b : _world->bodies())
    if (!b.moves && b.desc && in_world(b)) {
      avatar_static s;
      s.type = avatar_static::kind::body;
      s.body = b.desc;
      // Its surface as the relay's world has it (OW5h): the document's material, by value.
      s.material = std::make_shared<const cvc::physics::material>(
          _world->world().get_material(b.desc->material));
      _statics.push_back(std::move(s));
    }
  for (const std::string &id : _nodes) {
    std::uint64_t rev = 0;
    std::shared_ptr<const cvc::world::world_geometry> g = _world->world_geometry(id, &rev);
    if (g && !_world->bodies_of(id).empty()) {
      avatar_static s;
      s.type = avatar_static::kind::geometry;
      s.geometry = std::move(g);
      _statics.push_back(std::move(s));
      _taken[id] = rev;
    }
  }
  // The water (OW5j): its world's fluid fields, else the still water the relay's boats float on.
  for (const cvc::physics::field_snapshot &f : _world->world().snapshot().fields)
    if (f.desc.kind == cvc::physics::field_kind::fluid)
      _water.push_back(f.desc);
  std::shared_ptr<const cvc::nav::world_grid> grid;
  std::shared_ptr<const cvc::height_field> terrain;
  _world->vehicle_ground(grid, terrain);
  if (grid)
    if (const std::optional<cvc::physics::field> still =
            boat_still_water(app, *grid, _world->world()))
      _water.push_back(*still);
  ++_stamp.rev;
}

std::uint64_t propose_intent(holon &h, cvc::app &app, const std::string &room,
                             const std::string &self_pid, const avatar_intent &intent) {
  // The holon predicts with it from its next quantum; stamp it with that tick.
  h.set_intent(self_pid, intent);
  const std::uint64_t stamp = h.tick() + 1;
  write_intent(app, room, self_pid, intent, stamp);
  return stamp;
}

} // namespace ariadne
} // namespace cvc
