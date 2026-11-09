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

#ifndef __CVC_ARIADNE_WORLD_RULES_H__
#define __CVC_ARIADNE_WORLD_RULES_H__

#include <cvc/ariadne/avatar_physics.h> // avatar_body, avatar_energy_policy
#include <cvc/physics/types.h>
#include <map>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

struct LoadResult;
struct Scene;
struct Value;

// ----------------
// The world's rules (ARIADNE_WORLDS.md §3, WD2)
// ----------------
// What a room's avatars move under: the world document says it, the relay publishes it as four
// revisioned sections (room_rules.h), and every holon -- the relay's and each client's -- runs
// under the same one. Each section takes effect at its activation tick (holon.h).
//
//   physics      the world's gravity (from the document's scene: physics:)
//   archetypes   the bodies avatars may have, and the controllers each allows
//   policy       the movement veto (max_speed, allow_vertical) and the energy budget
//   controllers  what turns an intent into forces: a kernel by name and version, and its params
//
// Every avatar has the first archetype and moves with the first controller it allows (WD2; a
// participant's own pick comes with WD4 and WD6). default_world_rules() is what the conference
// had compiled in before: a 70 kg, 0.5 x 0.5 x 1.7 m person, walker@1, a 4 m/s cap.

struct world_physics {
  cvc::physics::vec3 gravity{0.0, 0.0, -9.80665}; // m/s^2
  bool operator==(const world_physics &o) const {
    return gravity.x == o.gravity.x && gravity.y == o.gravity.y && gravity.z == o.gravity.z;
  }
};

struct avatar_archetype {
  std::string id = "person";
  avatar_body body;
  std::vector<std::string> controllers{"walker@1"}; // controller ids; the first is the avatar's
  bool operator==(const avatar_archetype &) const = default;
};

struct avatar_controller {
  std::string id = "walker@1";
  std::string kernel = "walker";
  int version = 1;
  // The kernel's knobs. walker@1's walk_speed (m/s) is what a client's walk keys ask for; the run
  // a client may add is capped by the policy's max_speed.
  std::map<std::string, double> params{{"walk_speed", 2.0}};
  double param(const std::string &name, double def) const;
  bool operator==(const avatar_controller &) const = default;
};

struct avatar_policy {
  avatar_energy_policy energy;
  // The veto (holon::sanitize_intent): the fastest an intent may ask an avatar to move horizontally
  // (m/s), and whether it may ask to move vertically at all (avatars walk; gravity owns z).
  double max_speed = 4.0;
  bool allow_vertical = false;
  bool operator==(const avatar_policy &) const = default;
};

struct world_rules {
  world_physics physics;
  std::vector<avatar_archetype> archetypes{avatar_archetype{}};
  avatar_policy policy;
  std::vector<avatar_controller> controllers{avatar_controller{}};
  bool operator==(const world_rules &) const = default;

  // The archetype every avatar has: the first (the default person when there is none).
  const avatar_archetype &avatar() const;
  // The avatar's controller: the first its archetype allows, as `controllers` defines it (the
  // default walker@1 when that is missing).
  const avatar_controller &controller() const;
  const avatar_controller *find_controller(const std::string &id) const;
};

world_rules default_world_rules();

// Whether this build runs KERNEL@VERSION: walker@1 so far (WD4 brings a catalog).
bool controller_kernel_known(const std::string &kernel, int version);

// "" or what is wrong with RULES: an archetype's body that is not finite and positive, an id
// repeated or not a name, an archetype allowing no controller or one `controllers` does not define,
// a kernel this build does not run, a policy limit that is negative or not finite.
std::string check_world_rules(const world_rules &rules);

// The world: block (ARIADNE_WORLDS.md §2), on top of RULES:
//
//   world:
//     avatars:
//       archetypes:                 # replaces the default list; the first is every avatar's
//         person: { body: { mass: 70, width: 0.5, height: 1.7 }, controllers: [walker@1] }
//       policy:                     # each key given replaces the default
//         max_speed: 4.0
//         allow_vertical: false
//         energy: { enabled: true, force_max: 840, power_max: 1200, power_rest: 300,
//                   reserve_max: 30000, transport_cost: 0.2, empty: rest, recover_fraction: 0.25 }
//     controllers:                  # added to (or replacing, by id) the default walker@1
//       walker@1: { kernel: walker, version: 1, params: { walk_speed: 2.0 } }
//
// "" or what is wrong (check_world_rules included). A key it does not know is a warning, pushed
// onto WARNINGS when given, and so is avatars: catalog:, which the relay's seeded catalog still
// stands in for.
std::string parse_world_block(const Value &content, world_rules &rules,
                              std::vector<std::string> *warnings = nullptr);

// The world: block's parser (register_ari_block), process-wide; idempotent. Without it the loader
// skips a world: block. arisrv and ServerDocument register it.
void register_world_block();
// The world: block a load kept (register_world_block), or null.
const Value *find_world_block(const LoadResult &lr);

// A world document's rules: the defaults, its scene's physics: gravity, then its world: block
// (WORLD, null for none). A physics: fixed_dt other than the room's quantum (1/120 s), a backend
// or materials are warnings: the room's quantum is fixed, and bodies other than the avatars' wait
// for WD3. "" or what is wrong.
std::string world_rules_from(const Scene &scene, const Value *world, world_rules &out,
                             std::vector<std::string> *warnings = nullptr);

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_WORLD_RULES_H__
