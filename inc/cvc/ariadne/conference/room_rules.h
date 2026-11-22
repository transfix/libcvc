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

#ifndef __CVC_ARIADNE_CONFERENCE_ROOM_RULES_H__
#define __CVC_ARIADNE_CONFERENCE_ROOM_RULES_H__

#include <cstdint>
#include <cvc/ariadne/section.h>     // section_stamp
#include <cvc/ariadne/world_rules.h> // world_rules and its parts
#include <string>
#include <vector>

namespace cvc {
class app;
namespace ariadne {
namespace conference {

// ----------------
// The world's rules on the room's wire (ARIADNE_WORLDS.md §3, WD2)
// ----------------
// Four revisioned sections (section.h), each with its {epoch, at, rev} stamp, the revision last:
//
//   conf.rooms.<room>.physics.gravity.{x,y,z}
//   conf.rooms.<room>.avatars.archetypes.{count, <i>.{id, body.{mass, width, height},
//       controllers.{count, <j>}}}
//   conf.rooms.<room>.avatars.policy.{max_speed, allow_vertical, energy.{enabled, force_max,
//       power_max, power_rest, reserve_max, transport_cost, empty ("rest" | "stop"),
//       recover_fraction}}
//   conf.rooms.<room>.controllers.{count, <i>.{id, kernel, version, params.{names, <name>}}}
//
// Lists are indexed (a controller's id, `walker@1`, is no state name) and a reader takes `count`
// entries, so what a longer earlier version left behind is never read.
//
// AUTHORITY: the relay is the sole writer (room_write_gate refuses a client's write outside its
// own participant). Every holon, the relay's and each client's, runs under the same sections:
// holon_sync's pumps read them into the holon whenever a stamp moves, and the holon switches on
// each one's activation tick `at` (holon.h). A section the relay writes at start-up takes `at` 0:
// in force from the start. A change while the room runs takes a tick a little ahead
// (rules_activation_tick), so clients that hear of it in time switch with the relay, and the rest
// replay from it.

std::string room_physics_path(const std::string &room_id);
std::string avatar_archetypes_path(const std::string &room_id);
std::string avatar_policy_path(const std::string &room_id);
std::string room_controllers_path(const std::string &room_id);

// Each writer rewrites its whole section and commits it at AT (EPOCH 0: this process's), returning
// its stamp. Each reader fills OUT and returns true, or returns false (OUT untouched) when the
// section was never written.
section_stamp write_room_physics(cvc::app &app, const std::string &room_id, const world_physics &p,
                                 std::uint64_t at = 0, std::uint64_t epoch = 0);
section_stamp room_physics_stamp(cvc::app &app, const std::string &room_id);
bool read_room_physics(cvc::app &app, const std::string &room_id, world_physics &out);

section_stamp write_avatar_archetypes(cvc::app &app, const std::string &room_id,
                                      const std::vector<avatar_archetype> &a, std::uint64_t at = 0,
                                      std::uint64_t epoch = 0);
section_stamp avatar_archetypes_stamp(cvc::app &app, const std::string &room_id);
bool read_avatar_archetypes(cvc::app &app, const std::string &room_id,
                            std::vector<avatar_archetype> &out);

section_stamp write_avatar_policy(cvc::app &app, const std::string &room_id, const avatar_policy &p,
                                  std::uint64_t at = 0, std::uint64_t epoch = 0);
section_stamp avatar_policy_stamp(cvc::app &app, const std::string &room_id);
bool read_avatar_policy(cvc::app &app, const std::string &room_id, avatar_policy &out);

section_stamp write_room_controllers(cvc::app &app, const std::string &room_id,
                                     const std::vector<avatar_controller> &c, std::uint64_t at = 0,
                                     std::uint64_t epoch = 0);
section_stamp room_controllers_stamp(cvc::app &app, const std::string &room_id);
bool read_room_controllers(cvc::app &app, const std::string &room_id,
                           std::vector<avatar_controller> &out);

// All four, each committed at AT.
void write_room_rules(cvc::app &app, const std::string &room_id, const world_rules &r,
                      std::uint64_t at = 0, std::uint64_t epoch = 0);
// What the four say, with the default for any never written.
world_rules read_room_rules(cvc::app &app, const std::string &room_id);

// The world's rate (ARIADNE_WORLDS.md section 18.9), a section of its own:
//   conf.rooms.<room>.clock.hz   ticks a second (cvc::sim_rate_valid), the quantum 1/hz
// The relay writes it once at start-up, with its epoch, and runs its holon and the document's
// world at it. Every client's holon runs at it too: a host builds its holon from
// room_holon_config(app, room) and builds it again when the room's rate is not its holon's
// (holon_sync's on_room_clock: a relay restarted at another rate). A rate changes what every tick
// means, so it has no activation tick: it is the room's from the start of a relay's run. A relay
// from before the section writes none, and its room runs at 120 Hz.
std::string room_clock_path(const std::string &room_id);
section_stamp write_room_clock(cvc::app &app, const std::string &room_id, double hz,
                               std::uint64_t epoch = 0);
section_stamp room_clock_stamp(cvc::app &app, const std::string &room_id);
// The room's rate (Hz), or 0 when it was never written or is not a world's rate.
double read_room_clock(cvc::app &app, const std::string &room_id);

// The activation tick for a change the relay makes while the room runs at world tick NOW: half a
// second ahead (60 ticks of the 120 Hz holon), more than a client's lead plus the relay's 200 ms
// pass, so a client that is not far behind switches on the same tick as the relay, and a later
// one replays from it.
constexpr std::uint64_t kRulesLeadTicks = 60;
inline std::uint64_t rules_activation_tick(std::uint64_t now) { return now + kRulesLeadTicks; }

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_ROOM_RULES_H__
