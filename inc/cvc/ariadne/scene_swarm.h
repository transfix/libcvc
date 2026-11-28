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

// ----------------
// A swarm of navigating vehicles: the simulation of a `type: nav_swarm` scene node
// ----------------
// A document declares a swarm and the city it crosses; this runs it (docs/ARIADNE.md §5.4,
// "Swarms"). It is the simulation libcvc's C++ nav demos (nav_city_swarm and nav_city_drive,
// since retired) ran by hand, with the same steps:
//
//   * the city: a city bundle's world (a document's `source: { data }` dataset), rasterized onto
//     `grid` x `grid` cells over its bounds at the trained metric (0.05 navigator units a metre),
//     an arena the document draws up (`arena:`, swarm_arena: an open field, its walls, and the
//     phantom walls of a stale map of it), or, with neither, the synthetic city the trainer learns
//     on (cvc::nav::city_scene). Each is walled at its edge.
//   * the map the agents set out with: the city as it is, or, where the city has a stale map
//     (swarm_city::prior) and the `phantoms` key is on, that map, with walls the city lacks. They
//     learn the city as it is by sensing it, so a phantom wall stands in their belief until their
//     sensors' rays clear it.
//   * the navigator: cvc::nav::sim_world, its `agents` driven by the published trained CoefMLP
//     (coef_mlp::default_weights_path; the built-in biased net when none is installed) with the
//     trainer's vehicle (city_scene's rr, d_hat, dt and vmax), sensing the city into `belief`
//     planes under `fog` (shared: one plane; grouped: max(2, agents / 64); private: one each),
//     steering round one another (`separate`, sep_radius and sep_gain in vehicle radii) and
//     spawned on one edge with their goals on the other (`spawn: opposed`), or (`vehicles:`)
//     each where the document lists it, driving to its own goal.
//   * `route: belief` (GRL-SNAM's belief route): each agent drives an 8-connected A* route over
//     its own belief plane, its waypoints its goal in turn, planned again from where it is each
//     time a belief flip changes some plane's map. A believed wall bends the route from the
//     start; once sensing clears it, the route goes straight through where it stood.
//   * `targets` colour-coded rally points: each agent is sent to the target of its goal's
//     latitude band, each target the scattered goal nearest its band's centre, and each agent
//     wears its target's colour.
//   * the plant (`plant`): nav's kinematic bicycle, or a vehicle preset on the Jolt plant
//     (cvc::physics::plant_host: humvee, roboracer, a list, dynamic[:presets], ...).
//
// It steps on the app's world clock, one navigator step every veh.dt of world time (cvc::kSimTick,
// 7 quanta of the 1/120 s suite quantum), on a worker natively (cvc::nav::sim_thread, the app's
// compute pool for its kernels) and on the frame's thread in a browser without threads. The frame
// reads its latest frame (update(), now()).
//
// Its keys, under the key root (`<prefix>.swarm` for a node with no `keys:`), seeded from the
// node's props while unset:
//   agents belief fog plant seed     what the next build makes (`request: restart`, a
//   phantoms range                   sim.restart): agent count (with vehicles: listed, the first
//                                    that many of them), shared|grouped|private, 0|1, the plant's
//                                    name, the scatter's seed, whether the agents set out with
//                                    the city's stale map (1, where it has one) or with the city
//                                    as it is (0), and the sensor's range (world units)
//   separate sep_gain                inter-agent avoidance, on (1) or off, and its gain in vehicle
//                                    radii: live
//   follow                           the agent the view follows (-1: none): live
//   follow_at                        "x,y" (world): follow the agent nearest it; consumed
//   drag                             the target a minimap is dragging (-1: none); seeded, read by
//                                    no code here (swarm_minimap.ari's)
//   request                          restart | reseed (one more seed, then restart); consumed
//   targets.count, targets.<t>.{x,y,color}   the rally points: write x and y to move one, and its
//                                    agents drive to it
//   stats.*                          agents planes arrived tick driving policy city ready
//                                    building status vehicle_length vehicle_radius span
//                                    wall_height bounds ("min_x min_y min_z max_x max_y max_z",
//                                    a camera's frame request) center.{x,y,z}; time (world
//                                    seconds since the build), range (the sensor's, world
//                                    units), ghosts (cells some plane believes blocked that the
//                                    city is not; on the sense cadence) and ghosts0 (at the
//                                    build), replans (the times routes were planned since it)
//   follow_pose.{x,y,z,heading}      the followed agent, each frame
//
// One per swarm node, with nothing process-wide: two documents' swarms are two of these.

#ifndef __CVC_ARIADNE_SCENE_SWARM_H__
#define __CVC_ARIADNE_SCENE_SWARM_H__

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace cvc {

class app;
class height_field;

namespace world {
class world_geometry;
} // namespace world

namespace ariadne {

struct Value;

// A vehicle a document lists (`vehicles:`): where it starts and where it drives to, in world units,
// and its colour (unset: the targets' hsv colour for its place in the list).
struct swarm_vehicle {
  double start[2] = {0.0, 0.0};
  double goal[2] = {0.0, 0.0};
  float color[3] = {0.8f, 0.8f, 0.8f};
  bool has_color = false;
  std::string name;
};

// An arena a document draws up (`arena:`), in place of a city: an open field in world units
// (metres at the trained metric), walled at its edge. Boxes are x0 y0 x1 y1, each blocking the
// cells whose centres it covers (at least one row or column of them, however thin it is).
struct swarm_arena {
  double min_x = -100.0, min_y = -100.0, max_x = 100.0, max_y = 100.0;
  std::vector<std::array<double, 4>> walls;    // blocked in the arena and in its map
  std::vector<std::array<double, 4>> phantoms; // blocked in its map only: a stale map's walls
  double wall_height = 6.0;                    // the height its walls are drawn at
  std::string name = "the arena";
};

// What a swarm is built as: a node's props, or a host's own.
struct swarm_spec {
  int agents = 800;
  int grid = 96;                   // occupancy cells a side
  std::string belief = "grouped";  // shared | grouped | private
  bool fog = true;                 // sense the city into belief planes; false: the map is known
  unsigned seed = 7;               // the starts' and goals' scatter
  std::string spawn = "opposed";   // opposed (one edge to the other) | scattered
  int targets = 16;                // rally points, 1..64
  double reach = 0.8;              // arrival tolerance, navigator units
  bool separate = true;            // inter-agent avoidance
  double sep_radius = 3.0;         // its radius, vehicle radii
  double sep_gain = 1.5;           // its gain, vehicle radii
  std::string plant = "kinematic"; // plant_host's name
  // A .cvcnav to drive with; empty: the published trained CoefMLP (coef_mlp::default_weights_path),
  // else the built-in biased net; "default": the built-in biased net.
  std::string policy;
  int sense_every = 15; // navigator steps between the fog and discovered-wall repaints
  int sense_ticks = 4;  // navigator steps between senses (sim_world's sense_every)
  // The status line's words: what the agents are, and what the groups a target makes are (empty:
  // not counted): "800 agents ..." or "60 vehicles · 6 convoys ...".
  std::string noun = "agents";
  std::string groups;
  // The vehicles, listed: each starts and drives where it says, in place of the spawn's scatter
  // and the targets (each is its own target, in its own colour). agents defaults to how many.
  std::vector<swarm_vehicle> vehicles;
  // An arena, in place of a city (has_arena): the node's `arena:`.
  swarm_arena arena;
  bool has_arena = false;
  std::string route = "none"; // none | belief (this header's top)
  double range = 60.0;        // the sensor's range, world units (the range key's start)
  // The navigator on a thread of its own where there are threads (natively, a browser built with
  // them); false: on the frame's thread, as a browser without threads steps it. Not a prop.
  bool worker = true;

  // A node's props: agents, grid, belief, fog, seed, spawn, targets, reach, separate,
  // sep_radius, sep_gain, plant, policy, sense_every, noun, groups, vehicles (each {start: [x, y],
  // goal: [x, y], color, name}), arena ({bounds: [x0, y0, x1, y1] or size: [w, h] about the
  // origin, walls and phantoms: lists of [x0, y0, x1, y1], wall_height, name}), route, range,
  // sense_ticks.
  // Others are left as they are.
  static swarm_spec from_props(const Value &props);
};

// The city a swarm crosses: its occupancy (row 0 at min_y, column 0 at min_x; non-zero blocked)
// over world bounds, the navigator's metre, and its ground.
struct swarm_city {
  std::vector<std::uint8_t> occ;
  int rows = 0, cols = 0;
  double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
  double min_z = 0.0, max_z = 0.0; // its ground's lowest point and its highest (a building's top)
  double scale = 0.05;             // navigator units a world unit
  std::shared_ptr<const cvc::height_field> terrain; // null: flat at 0
  bool real = false;  // a city bundle's (its data node draws it); false: the swarm draws it
  bool arena = false; // a document's arena: in metres, so its vehicles are a Humvee long
  std::string name;
  // The map its agents set out with, [rows*cols] (empty: occ, the city as it is): occ and the walls
  // a stale map shows that the city lacks, its phantoms (cells blocked here and free in occ).
  std::vector<std::uint8_t> prior;
  double wall_h = 0.0; // the height its walls are drawn at; 0: 0.06 of its span

  double span() const { return max_x - min_x; }
  // The ground at (x, y), clamped to the bounds; 0 without a terrain or over a hole.
  double ground(double x, double y) const;
};

// The synthetic city (cvc::nav::city_scene(grid)), walled at its edge.
swarm_city synthetic_swarm_city(int grid);
// A city bundle's world, rasterized onto grid x grid cells over its bounds (cvc::nav::rasterize:
// its buildings' footprints and any hard ground) and walled. A world without buildings keeps the
// synthetic city's occupancy over its bounds, as the demos did. Empty (rows 0) without bounds.
swarm_city swarm_city_from_world(const cvc::world::world_geometry &world, int grid,
                                 const std::string &name = std::string());
// An arena on a grid of square cells, its longer side `grid` of them (the far edges moved to the
// last cell's centre), walled at its edge, its prior its occupancy and its phantoms when it has
// any.
swarm_city arena_swarm_city(const swarm_arena &arena, int grid);

class SceneSwarm {
public:
  // `keys` is the absolute state path of its keys. Nothing runs until set_city().
  SceneSwarm(cvc::app &app, std::string keys, swarm_spec spec);
  ~SceneSwarm();
  SceneSwarm(const SceneSwarm &) = delete;
  SceneSwarm &operator=(const SceneSwarm &) = delete;

  // Cross `city` from now on: the swarm is built again on it (off the frame's thread where there
  // are threads; the old one runs until the new one is in). Owner thread.
  void set_city(swarm_city city);
  bool has_city() const;
  const swarm_city &city() const;

  // Once a frame on the owner thread, after the frame drove the world clock: the control keys
  // taken, a world built since swapped in, the steps due run (on this thread without a worker),
  // and the latest frame read. `wait` blocks until the worker has run every step due (a capture).
  // True when now() is a newer frame.
  bool update(bool wait = false);

  // The latest frame: poses in world metres, the ground under each agent, its carrot mode (0 seek,
  // 1 wall-follow) and whether it is at its goal.
  struct frame {
    long tick = 0; // navigator steps since the build
    int n = 0;
    std::vector<float> pos;     // [2n] x, y
    std::vector<float> heading; // [n] rad
    std::vector<int> mode;      // [n]
    std::vector<std::uint8_t> reached;
    std::vector<double> ground; // [n] z
    std::vector<float> carrot;  // [2n] world: each agent's live carrot, what it steers at
    long arrived = 0;
  };
  const frame &now() const;

  // Bumps each time a new world is in: the agents, their colours, goals and planes may all differ.
  std::uint64_t generation() const;
  bool running() const;  // a world is in
  bool building() const; // a world is being built
  int size() const;      // agents
  int planes() const;    // belief planes
  bool fog() const;      // this world senses (its belief planes differ from the map)

  // The vehicle: its radius in navigator units, and its length in world units, the length the
  // demos drew it at (a real 4.72 m Humvee on a real city, 2 radii on the synthetic one).
  double vehicle_radius() const;
  double vehicle_length() const;
  // The height the city's walls are drawn at: its wall_h, else 0.06 of its span (the synthetic
  // city's blocks).
  double wall_height() const;
  // The sensor's range in world units, as this world was built (the range key).
  double sensor_range() const;

  // What the view draws, for this generation.
  const std::vector<float> &colors() const;        // [3n] each agent's target colour
  const std::vector<float> &goals() const;         // [2n] world
  const std::vector<int> &agent_targets() const;   // [n]
  const std::vector<float> &target_colors() const; // [3T]
  const std::vector<float> &targets() const;       // [2T] world, as moved
  std::uint64_t targets_revision() const;          // bumps when one moves
  const std::vector<float> &plane_colors() const;  // [3M] the mean colour of each plane's agents

  // The fog, on the sense cadence: per cell 0 never seen, 1 seen before, 2 in some agent's view;
  // and, for each wall cell that some plane has seen, the planes that have (at most 128 each).
  std::uint64_t coverage_revision() const;
  const std::vector<std::uint8_t> &coverage() const;
  struct sensed_wall {
    int cell = 0;
    std::vector<int> planes;
  };
  const std::vector<sensed_wall> &sensed_walls() const;
  // What the planes believe, on the same cadence: per cell 0 no plane's map blocks it, 1 some
  // plane's map blocks it and so does the city (a wall it knows of), 2 some plane's map blocks it
  // and the city does not (a ghost: a stale map's phantom not yet sensed clear). ghosts() counts
  // the 2s, and ghosts_at_build() counted them as the world was built.
  const std::vector<std::uint8_t> &belief() const;
  int ghosts() const;
  int ghosts_at_build() const;

  // With route: belief (empty otherwise): each agent's route in world units, x, y pairs from its
  // first waypoint to its goal, and the waypoint it drives to now (an index into it).
  // routes_revision() bumps each time they are planned.
  const std::vector<std::vector<float>> &routes() const;
  const std::vector<int> &route_cursor() const;
  std::uint64_t routes_revision() const;

  // The agent nearest a world point (-1 with none).
  int nearest_agent(double x, double y) const;
  // Move target t to a world point: its agents drive to it.
  void move_target(int t, double x, double y);
  // Build the swarm again now, from its keys (agents, belief, fog, plant, seed).
  void restart();

  const std::string &keys() const;

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_SCENE_SWARM_H__
