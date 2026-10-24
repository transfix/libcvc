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
  bool on() const { return horizon_s > 0.0f; }
};

// Agent i's shift of its carrot from the intents it heard (its own intent `self`).
struct yield_shift {
  float dx = 0.0f, dy = 0.0f;
  int conflicts = 0; // intents it reacted to
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
