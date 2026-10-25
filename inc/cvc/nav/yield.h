/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav::yield_step: agents that get out of each other's way, from the intents each has heard
// (nav/intent.h). sim_world::set_yield runs it every tick, after the carrot FSM and before the
// drive, as a shift of each agent's carrot (its steering target), so it works on every drive and
// on a plant.
//
// For agent i and each intent j it has heard (j's pose carried forward by j's speed over the
// intent's age, its path with it):
//   1. Out of the way. If i is within `clearance` of j's path, ahead of j and reached within the
//      horizon, i moves off it: its carrot shifts away from the path's nearest point by
//      gain x (clearance - distance) x urgency x w, with urgency = 1 - (arrival time / horizon)
//      and w = resolver(i, j), the share of the conflict i takes. On the path itself it moves to
//      j's right. Near along the path (within `near_dist`) counts whatever the speeds: urgency is
//      then at least 1 - (distance / near), so a column standing still clears for a lead starting
//      from rest behind it, where arrival times are long.
//   2. Around. If j sits within `clearance` of i's own path, ahead of i and reached within the
//      horizon, i steers round it: the same shift, away from j, by pass_gain and the share i
//      cannot count on j to make, 1 - cooperation x resolver(j, i). On the path, to j's left as i
//      sees it (j moving right, i passing left).
// A shift that would put the carrot in a wall (the field's phi there below the agent's radius) is
// tried on the other side, and dropped if that side is no better.
//
// An agent the host commands itself (sim_world::override_command: a route_follower's manoeuvre
// or pursuit, a host's own pure pursuit) has no carrot to shift. With steer_overrides, a forward
// override's steer takes pure pursuit's correction toward the same shift instead,
// atan(2 L y / d^2), y the shift across the heading and d the lookahead (steer_look_s of its
// speed, within 2 x clearance .. near_dist), within the lock. Backing up is left as commanded.
//
// Two more, on the speed (sim_world: every agent on a plant, an override on the kinematic arm):
//   - Passing: with another within clearance of its path s ahead, an agent drives no faster than
//     (s - 2.5 rr) / pass_time_s, so it reaches it no sooner than the time that one has to clear,
//     and stops short of it, by the two bodies and a margin, if it does not. A host driving it on
//     a route then plans round it (sim_world::yield_corridors, yield_pass_distance). With
//     pass_hold_s > 0 (opt-in), a hold that cannot end, behind one standing still itself, is
//     released after that long: the higher priority (then the lower index) of the two first, so
//     two holding each other do not both drive. Off by default: released, it drives on into the
//     one ahead, the push the cap stops; a host's replanning is the way round.
//   - Backing: an agent backing up stops short of one behind it, by the same 2.5 rr, with the
//     same cap on its reverse speed; yielding otherwise leaves backing up as commanded.
//   - Creep (opt-in): an agent that must move out of another's path (rule 1, at least creep_need
//     of it) but stands still or holds cannot steer out of it; with creep_speed > 0 it creeps
//     forward at that speed so its steer can, within the clearance its footprint has to the walls.
//     Off by default: forward is often along the very path it should leave.
//
// And a route planner can keep off the paths others mean to drive: sim_world::yield_corridors
// gives them as soft costs for plan_vehicle_route (route_options::avoid).
//
// So a convoy's lead coming up behind its followers announces its path; the followers pull over
// to the right of it and the lead passes on the left, instead of pushing through. Equal
// priorities share a conflict: two agents head-on each move half out of the way.

#ifndef __CVC_NAV_YIELD_H__
#define __CVC_NAV_YIELD_H__

#include <cvc/nav/drive.h>
#include <cvc/nav/intent.h>
#include <vector>

namespace cvc {
namespace nav {

struct yield_config {
  float horizon_s = 0.0f;   // s: how far ahead intents reach and conflicts count; 0 = off
  float clearance = 0.0f;   // normalized: the corridor's half-width; 0 = 3 x the vehicle's rr
  float gain = 1.0f;        // the carrot's shift out of another's path, per unit of intrusion
  float pass_gain = 1.0f;   // and round one in its own path
  float cooperation = 0.5f; // [0, 1]: how far a passer counts on the other moving
  float min_len = 0.0f;     // normalized: an intent's shortest path; 0 = 4 x clearance
  float min_speed = 0.05f;  // normalized / s: the floor arrival times are taken at
  float near_dist = 0.0f;   // normalized: along a path this close a conflict counts at any speed;
                            // 0 = 4 x clearance, < 0 = off (arrival times alone)
  bool steer_overrides = true; // a forward override_command's steer takes the shift too
  float steer_look_s = 0.5f;   // s of speed the override correction's lookahead is
  float pass_time_s = 1.5f;    // s: the passing speed cap's; 0 = off
  float pass_hold_s = 0.0f;    // s held behind one standing still before release; 0 = never
  float creep_speed = 0.0f;    // normalized / s; <= 0 = no creep (the default)
  float creep_need = 0.0f;     // normalized: the rule-1 shift that calls for it; 0 = the rr
  bool on() const { return horizon_s > 0.0f; }
};

// Agent i's shift of its carrot from the intents it heard (its own intent `self`).
struct yield_shift {
  float dx = 0.0f, dy = 0.0f;
  int conflicts = 0;                  // intents it reacted to
  float out_dx = 0.0f, out_dy = 0.0f; // the part of the shift out of others' paths (rule 1)
  float pass_s = -1.0f; // along its path to the nearest one it passes (rule 2); -1 none
  int pass_j = -1;      // that one's agent index
};

// The distance from (px, py) to the polyline from (x0, y0) through path[0..n), the arc length to
// its nearest point (s), that point (qx, qy) and the path's unit tangent there (tx, ty). A path
// with no points is the single point (x0, y0), with the tangent (cos h, sin h).
float intent_path_distance(float x0, float y0, float h, const float *path, int n, float px,
                           float py, float &s, float &qx, float &qy, float &tx, float &ty);

// i's shift from what it heard (one pass over `heard`; no wall check). `clearance` is resolved
// (> 0).
yield_shift yield_agent(const yield_config &c, float clearance, const agent_intent &self,
                        const std::vector<agent_intent> &heard, double t,
                        const yield_resolver &resolve);

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_YIELD_H__
