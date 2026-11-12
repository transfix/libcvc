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
#include <functional>
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
// world's roads. One per WorldVehicles, with nothing process-wide.
//
// Under its key (<prefix>.spawns) it publishes the points of the world on (`ids`, `count`,
// `summary`, `points.<id>.{x, y, heading, team, archetype, world, free}`), the suggestions
// (`suggestions.{count, summary}`), `policy`, `scope`, `status` and `rev`. It follows:
//   tool                   add, remove or tag: what a pick does
//   pick.*                 a viewport's pick (input: pick, pick: <prefix>.spawns.pick): on a
//                          press that hit, the tool runs at the point
//   heading                the heading (degrees) a new point takes
//   tag.team, tag.archetype   what `tag` (and a new point) gives a point
//   policy, scope          the policy and its scope
//   suggest.request        suggest points along the roads (suggest.count of them, suggest.spacing
//                          metres apart); accept.request adds them; clear.request removes every
//                          point of the world on
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

  // A spawn's start, by the policy: from the points of the world on whose tags match (a team of
  // -1 and an archetype of "" match anything) and that are free in the grid. `crowded(x, y, r)`
  // says whether a vehicle is within r metres. A point a vehicle sits on moves aside along its
  // heading's normal. False (and nothing set) when no point applies.
  bool choose(int team, const std::string &archetype,
              const std::function<bool(double x, double y, double r)> &crowded, double &x,
              double &y, double &heading_rad);

  // Edits: a point's id (0 and why when it is refused: off the grid, blocked, or too near
  // another), and removing and tagging one. A new point is on the world on.
  int add(spawn_point p, std::string *why = nullptr);
  bool remove(int id);
  bool tag(int id, int team, const std::string &archetype);
  void clear(); // every point of the world on
  // The point nearest (x, y) within r metres, or 0.
  int nearest(double x, double y, double r) const;

  // `n` points along the world's roads, `spacing` metres apart, free in the grid, each heading
  // along its road (none without roads). suggest() keeps them as the suggestions; accept() adds
  // them as points.
  std::vector<spawn_point> suggest(int n, double spacing);
  int accept();

  void poll();    // the keys above
  void publish(); // now

  struct entry {
    int id = 0;
    spawn_point point;
    double z = 0.0; // the grid's ground under it
    bool free = true;
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

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_WORLD_SPAWNS_H__
