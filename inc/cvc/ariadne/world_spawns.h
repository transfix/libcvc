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

#ifndef __CVC_ARIADNE_WORLD_SPAWNS_H__
#define __CVC_ARIADNE_WORLD_SPAWNS_H__

#include <cstdint>
#include <cvc/ariadne/world_rules.h> // spawn_point, spawn_rules
#include <cvc/nav/plant_types.h>     // plant_medium
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace nav {
struct world_grid;
}
namespace world {
class world_geometry;
}

namespace ariadne {

// ----------------
// Where a world's vehicles start (ARIADNE_WORLDS.md §14, L3)
// ----------------
// The world runtime's spawn points: the document's (world: spawns:) and the ones a minimap adds,
// each on one of the scene's worlds or on all of them, checked against the grid of the world that
// is on; the policy a spawn without a place of its own follows; and points suggested along the
// world's roads, or over its water for a boat. One per WorldVehicles, with nothing process-wide.
//
// Each archetype starts on its medium (set_needs; spawn_rules' ford_depth, keel_clearance and
// shore_clearance): a land vehicle where the grid is free and the water no deeper than a ford, a
// boat where the water is deep enough for its draft and open for half its length round it. A
// point serves an archetype only where its medium allows: an untagged point on the shore serves
// the cars, one in a lake the boats.
//
// Under its key (<prefix>.spawns) it publishes the points of the world on (`ids`, `count`,
// `summary`, `points.<id>.{x, y, heading, team, archetype, world, free, medium}`), the suggestions
// (`suggestions.{count, summary}`), `policy`, `scope`, `status` and `rev`. It follows:
//   tool                   add, remove or tag: what a pick does
//   pick.*                 a viewport's pick (input: pick, pick: <prefix>.spawns.pick): on a
//                          press that hit, the tool runs at the point
//   heading                the heading (degrees) a new point takes
//   tag.team, tag.archetype   what `tag` (and a new point) gives a point
//   policy, scope          the policy and its scope
//   suggest.request        suggest points along the roads (suggest.count of them, suggest.spacing
//                          metres apart), or over the water when tag.archetype is a boat;
//                          accept.request adds them; clear.request removes every point of the
//                          world on

// What an archetype needs of the place it starts: its medium, and a boat's draft (m below the
// waterline, its keel or its propeller's) and half its hull's length (m).
struct spawn_need {
  cvc::nav::plant_medium medium = cvc::nav::plant_medium::land;
  double draft = 0.0;
  double half_length = 0.0;
};

class WorldSpawns {
public:
  // `key` is an absolute state path ("" = no keys); `app` may be null.
  WorldSpawns(cvc::app *app, spawn_rules rules, std::string key);
  ~WorldSpawns();
  WorldSpawns(const WorldSpawns &) = delete;
  WorldSpawns &operator=(const WorldSpawns &) = delete;

  // The world that is on, its grid and its world (for its roads; null for a terrain or a flat
  // ground). The points of another world leave the table; each point's `free` is checked again.
  void set_world(const std::string &name, std::shared_ptr<const nav::world_grid> grid,
                 std::shared_ptr<const cvc::world::world_geometry> world);
  // Each archetype's need, by id (one not named starts on land). Each point's `free` is checked
  // again.
  void set_needs(std::map<std::string, spawn_need> needs);
  spawn_need need_of(const std::string &archetype) const;

  // The ground an archetype starts on: the world on's grid for its medium (nav::medium_grid: land
  // with the water deeper than ford_depth blocked, or the water at least its draft and
  // keel_clearance deep), that grid's signed distance, and the clearance (m) a start keeps from
  // what it blocks (on water half its hull's length and shore_clearance, at least the land's). Null
  // grid without one. Kept until the world or the needs change.
  struct start_ground {
    std::shared_ptr<const nav::world_grid> grid;
    std::shared_ptr<const std::vector<float>> sdf;
    double clearance = 0.0;
    // Whether (x, y) is on the grid, free and that clear.
    bool free_at(double x, double y) const;
    // Whether any sample is free and that clear.
    bool any() const;
  };
  start_ground ground_for(const std::string &archetype) const;
  // The same for a need.
  start_ground ground_for(const spawn_need &need) const;

  // A spawn's start, by the policy: from the points of the world on whose tags match (a team of
  // -1 and an archetype of "" match anything) and that are free for the archetype's medium
  // (ground_for). `crowded(x, y, r)`
  // says whether a vehicle is within r metres. A point a vehicle sits on moves aside along its
  // heading's normal. False (and nothing set) when no point applies.
  bool choose(int team, const std::string &archetype,
              const std::function<bool(double x, double y, double r)> &crowded, double &x,
              double &y, double &heading_rad);

  // Edits: a point's id (0 and why when it is refused: off the grid, blocked or not on its
  // archetype's medium (an untagged one: on no archetype's), or too near another), and removing
  // and tagging one. A new point is on the world on.
  int add(spawn_point p, std::string *why = nullptr);
  bool remove(int id);
  bool tag(int id, int team, const std::string &archetype);
  void clear(); // every point of the world on
  // The point nearest (x, y) within r metres, or 0.
  int nearest(double x, double y, double r) const;

  // `n` points `spacing` metres apart for the archetype `tag.archetype` names: along the world's
  // roads, free on land and out of the water, each heading along its road (none without roads); or
  // for a boat, over the water deep and open enough for it, spread over its bodies of water.
  // suggest() keeps them as the suggestions; accept() adds them as points.
  std::vector<spawn_point> suggest(int n, double spacing);
  int accept();

  void poll();    // the keys above
  void publish(); // now

  struct entry {
    int id = 0;
    spawn_point point;
    double z = 0.0; // the grid's ground under it
    bool free = true;   // for its archetype (untagged: for any archetype)
    std::string medium; // what it is free for: land, water, both, or "" (neither)
  };
  // The grid's ground at (x, y) (0 without a grid or a terrain).
  double ground(double x, double y) const;
  // The points of the world on, in the order they were added.
  std::vector<entry> points() const;
  const std::vector<spawn_point> &suggestions() const;
  std::string policy() const;
  std::string scope() const;
  // Bumps on every change to the points, their tags or the suggestions (a view draws them again).
  std::uint64_t revision() const;
  const std::string &status() const;

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

// Points along `world`'s roads (its items on asphalt): one per `spacing` metres square at most,
// at least `clearance` from anything blocked in `grid`, each heading along its road, at most `n`,
// the same on every platform.
std::vector<spawn_point> road_spawn_points(const cvc::world::world_geometry &world,
                                           const nav::world_grid &grid, int n, double spacing,
                                           double clearance);

// Points over the free samples of `grid` (a boat's: nav::medium_grid on water) at least
// `clearance` from anything blocked, one per `spacing` metres square at most, at most `n`, each
// heading along the shore: the bodies of water with room for one each get one in turn, so a small
// lake is not left out for a sea's many. The same on every platform.
std::vector<spawn_point> water_spawn_points(const nav::world_grid &grid, int n, double spacing,
                                            double clearance);

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_WORLD_SPAWNS_H__
