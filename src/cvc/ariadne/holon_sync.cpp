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
#include <cvc/ariadne/conference/room_intrinsics.h> // room_participant_path, room_roster
#include <cvc/ariadne/conference/room_scene.h>      // read_scene_objects, scene_revision
#include <cvc/ariadne/holon.h>                      // cvc::ariadne::holon (server_pump)
#include <cvc/ariadne/holon_sync.h>
#include <cvc/core/app.h>
#include <cvc/physics/types.h> // vec3, quat
#include <cvc/state/state.h>   // cvc::state, SEPARATOR, findDescendant
#include <initializer_list>
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
  if (out_tick) {
    cvc::state *t = app.root().findDescendant(leaf(room, pid, {"intent", "tick"}));
    *out_tick = t ? std::strtoull(t->value().c_str(), nullptr, 10) : 0;
  }
  return true;
}

holon_config room_holon_config() {
  holon_config cfg;
  cfg.physics = true;
  cfg.max_speed = 4.0; // twice the walk; room to add a run without loosening the veto
  return cfg;
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
// Physics: reload the statics when the room scene changed (its revision moved).
void sync_statics(holon &h, cvc::app &app, const std::string &room) {
  if (!h.physics())
    return;
  const std::uint64_t rev = conference::scene_revision(app, room);
  if (rev != 0 && rev != h.statics_revision())
    h.set_statics(room_statics(app, room), rev);
}
} // namespace

void server_pump(holon &h, cvc::app &app, const std::string &room, double wall_dt) {
  sync_statics(h, app, room);
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
  // One authoritative frame.
  h.advance(wall_dt);
  // Publish each participant's authoritative pose (these are local writes on the
  // relay's shard; the caller fans them to the clients): what changed, and a
  // heartbeat of about a second for an avatar standing still.
  const std::uint64_t heartbeat =
      std::max<std::uint64_t>(1, static_cast<std::uint64_t>(1.0 / h.clock().fixed_dt()));
  for (const conference::room_participant &p : roster)
    write_pose(app, room, p.pid, h.state(p.pid), h.tick(), heartbeat);
}

void client_pump(holon &h, cvc::app &app, const std::string &room, const std::string &self_pid,
                 double wall_dt) {
  sync_statics(h, app, room);
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
    if (!h.has_participant(p.pid))
      h.add_participant(p.pid, /*local=*/p.pid == self_pid, have ? s : cvc::physics::body_state{},
                        /*pose_known=*/have);
    if (have)
      h.apply_authoritative(p.pid, s, server_tick);
  }
  for (const std::string &pid : h.participants())
    if (!present.count(pid))
      h.remove_participant(pid);
  h.advance(wall_dt);
}

void propose_intent(holon &h, cvc::app &app, const std::string &room, const std::string &self_pid,
                    const avatar_intent &intent) {
  // The holon predicts with it from its next quantum; stamp it with that tick.
  h.set_intent(self_pid, intent);
  write_intent(app, room, self_pid, intent, h.tick() + 1);
}

} // namespace ariadne
} // namespace cvc
