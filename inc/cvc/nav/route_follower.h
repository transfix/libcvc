/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav::route_follower: drives a sim_world's agents along vehicle routes
// (plan_vehicle_route), reverse legs and cusps included (the vehicle-physics plan's P9 R2).
//
//   cvc::nav::route_follower follow(sim, opts);
//   follow.set(i, cvc::nav::plan_vehicle_route(...)); // in the sim's world units
//   for (;;) {
//     follow.apply(); // before each step
//     sim.step();
//   }
//
// A route splits into legs at its cusps; each leg is driven in one gear. Each apply(), for every
// agent with a route:
//   - Its progress is the nearest pose of the current leg, ahead of where it last was.
//   - A forward leg that runs to the route's end is the trained drive's: the agent is retargeted
//     to a carrot `lookahead` along the route (at least twice the sim's reach_tol, or the drive
//     would park at it), the farthest it can see on its own belief plane (a carrot behind a
//     corner stalls the drive against the building), so it keeps off walls and parks at the end.
//     The carrot moves on only once the agent is halfway to it, since a retarget resets the drive's
//     carrot state. Where it cannot see that far (a tight corner, a wall alongside), it takes
//     that stretch geometrically, as below, until it can. The leg is done within reach_tol of
//     its end, where the drive parks.
//   - A leg that ends at a cusp (backing up, or the forward leg of a multi-point turn) is driven
//     geometrically: pure pursuit along the leg in its gear at `manoeuvre_speed`, slowing to stop
//     at the cusp, through sim_world::override_command.
//   - At a cusp, once stopped, the agent starts the next leg.
//   - A route that ends backing up stops at its end; the agent is then retargeted there, so the
//     drive holds it.
// Routes are in the sim's world units (the occupancy grid's frame), as are the options' lengths
// and speeds; the follower converts to nav's normalized frame.

#ifndef __CVC_NAV_ROUTE_FOLLOWER_H__
#define __CVC_NAV_ROUTE_FOLLOWER_H__

#include <cvc/nav/vehicle_route.h>
#include <vector>

namespace cvc {
namespace nav {

class sim_world;

struct follower_options {
  double wheelbase = 0.0;       // world units: pure pursuit's steer; 0 = the sim's veh.L
  double max_steer = 0.0;       // rad; 0 = the sim's veh.delta_max
  double lookahead = 0.0;       // world units ahead along the route; 0 = 3 x the sim's reach_tol
                                // (at least 3 route steps). Never under 2 x reach_tol.
  double manoeuvre_speed = 1.5; // world units / s, in either gear
  double pursuit_speed = 5.0;   // world units / s, the drive's leg where it is taken geometrically
  double accel = 1.0;           // world units / s^2 a manoeuvre speeds up and slows down at
  double stop_speed = 0.15;     // world units / s that counts as stopped at a cusp
  double cusp_tolerance = 0.0;  // how near a cusp counts as there; 0 = a route step
};

class route_follower {
public:
  explicit route_follower(sim_world &sim, follower_options options = {});

  // Agent i follows route r (replacing any it had); an empty route clears it.
  void set(int i, const vehicle_route &r);
  void clear(int i);
  bool following(int i) const;
  // Its state: the leg it is on and how many legs the route has (0 without a route), whether it
  // is backing up now, and whether it has finished the route (the last leg's end, stopped).
  int leg(int i) const;
  int legs(int i) const;
  bool reversing(int i) const;
  bool done(int i) const;

  // Before each sim.step(): retarget or command every following agent.
  void apply();

private:
  struct agent {
    std::vector<route_pose> poses;
    std::vector<int> leg_end; // each leg's last pose index (a cusp, or the route's end)
    int leg = 0, progress = 0;
    int carrot = -1; // the drive leg's carrot, a pose index; -1 = none yet
    bool done = false;
  };
  sim_world &_sim;
  follower_options _o;
  std::vector<agent> _agents;
};

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_ROUTE_FOLLOWER_H__
