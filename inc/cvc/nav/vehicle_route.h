/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav::plan_vehicle_route: a route a real vehicle can drive, through the navigator's
// occupancy grid (the vehicle-physics plan's P9).
//
// grid_nav's astar plans for a point that turns on the spot. A vehicle has a turning circle, a
// body and a reverse gear. A Humvee needs a 6.7 m circle and is 4.8 m long, and in a street
// narrower than that circle it cannot follow an 8-connected path round a corner, or turn back.
// This planner searches over the vehicle's pose (x, y, heading) instead (Hybrid A*: Dolgov,
// Thrun, Montemerlo and Diebel, 2008):
//   - each move is an arc of `step` world units at a curvature from -1 to +1 times
//     1 / min_radius, forward or, if allowed, backing up;
//   - a pose is free when every footprint disc keeps clear of the walls (the grid's exact
//     distance transform, sampled between cells);
//   - the cost is the length driven, with backing up dearer (reverse_cost), each change of
//     direction dearer still (cusp_cost), a little for steering and for changing the steer,
//     and for coming closer to a wall than the comfort distance;
//   - the heuristic is the shortest 8-connected distance to the goal through free cells, which
//     never overestimates.
// A dead end or a corner too tight to take forward therefore comes back as a multi-point turn:
// poses marked `reverse` between cusps.
//
// The route starts at the vehicle's pose even when its body already touches the comfort margin
// or a wall (a vehicle wedged in a corner): its first `escape` moves may stay as close as the
// start was, so the route can lead it out.
//
// Grid convention, as sim_world's and grid_nav's: `occ` is row-major rows x cols, nonzero =
// blocked, cell (r, c) centred at (min_x + c * (max_x - min_x) / (cols - 1),
// min_y + r * (max_y - min_y) / (rows - 1)), row 0 at min y. The cells should be square. Poses
// and lengths are in the grid's world units; headings in radians, counter-clockwise from +x.

#ifndef __CVC_NAV_VEHICLE_ROUTE_H__
#define __CVC_NAV_VEHICLE_ROUTE_H__

#include <cstdint>
#include <vector>

namespace cvc {
namespace nav {

// The vehicle, in world units. The pose a route plans is the rear axle's, the point nav's
// bicycle and the physics plant's readback track.
struct route_vehicle {
  double min_radius = 0.0; // the rear axle's tightest turn: wheelbase / tan(max steer); 0 = any
  // The footprint: discs of disc_radius at these offsets along the heading from the rear axle
  // (sim_world::plant_plan_of gives a plant vehicle's). Empty = one disc at the rear axle.
  std::vector<double> disc_offsets;
  double disc_radius = 0.0;
  bool reverse = true;       // may it back up
  double reverse_cost = 2.0; // a unit of length backing up costs this many going forward
  double cusp_cost = 0.0;    // world units of length a change of direction costs; 0 = 4 x step
};

struct route_options {
  double step = 0.0;           // arc length of a move; 0 = 1.5 cells, at most 0.6 x min_radius
  int headings = 72;           // heading bins in the search's closed set (5 degrees)
  double goal_tolerance = 0.0; // how near the rear axle must come to the goal; 0 = 2 cells
  double margin = 0.0;         // world units the discs keep from walls beyond their radius
  double comfort = 0.0;        // nearer than this beyond the margin costs more; 0 = disc_radius
  double comfort_cost = 0.5;   // the extra cost per unit length at a wall, falling to 0 at comfort
  int escape = -1;             // moves a wedged start may stay as close as it is; < 0 = auto
  // The heuristic's weight (weighted A*): a route at most this many times dearer than the
  // cheapest, found with far fewer expansions; 1 searches for the cheapest. Across a 384 m city
  // (nav_vehicle_route_test) 1.5 found a route 1.9% longer in 740 expansions and 10 ms, where 1
  // took 168,000 expansions and 0.66 s.
  double heuristic_weight = 1.5;
  int max_expansions = 200000;
};

struct route_pose {
  double x = 0.0, y = 0.0, heading = 0.0;
  bool reverse = false; // reached backing up (the start: as the route leaves it)
};

struct vehicle_route {
  bool reached = false;          // false: no route within max_expansions (poses then end at the
                                 // pose nearest the goal by the heuristic)
  std::vector<route_pose> poses; // from the start, one per move
  int cusps = 0;                 // changes of direction
  double length = 0.0;           // driven, either way
  double reverse_length = 0.0;   // of which backing up
  int expansions = 0;
};

vehicle_route plan_vehicle_route(const std::uint8_t *occ, int rows, int cols, double min_x,
                                 double min_y, double max_x, double max_y, const route_vehicle &v,
                                 const route_pose &start, double goal_x, double goal_y,
                                 const route_options &o = {});

// The footprint's clearance at a pose: the least distance from a disc's edge to a wall (world
// units; negative inside one), on the same distance field the planner samples. For checking a
// route, or where a vehicle stands.
double route_clearance(const std::uint8_t *occ, int rows, int cols, double min_x, double min_y,
                       double max_x, double max_y, const route_vehicle &v, const route_pose &p);

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_VEHICLE_ROUTE_H__
