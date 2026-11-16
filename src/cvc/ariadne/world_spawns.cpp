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
#include <array>
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/world_spawns.h>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/nav/world_grid.h>
#include <cvc/state/state.h>
#include <cvc/world/surface.h>
#include <cvc/world/world_geometry.h>
#include <deque>
#include <exception>
#include <map>
#include <set>
#include <utility>

namespace cvc {
namespace ariadne {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kClearance = 2.0; // m a point keeps from anything blocked
constexpr double kNear = 6.0;      // m a pick reaches to the nearest point; points keep apart

std::string num(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.6g", v);
  return buf;
}

std::string fixed(double v, int digits) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.*f", digits, v);
  return buf;
}

// splitmix64: the same draw on every platform.
std::uint64_t mix(std::uint64_t x) {
  x += 0x9e3779b97f4a7c15ull;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ull;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebull;
  return x ^ (x >> 31);
}

// (x, y) rotated by the unit quaternion q (w, x, y, z) about z only matters for a road item's
// plan; apply it whole and keep x, y.
std::array<double, 3> rotate(const std::array<double, 4> &q, double x, double y, double z) {
  const double w = q[0], qx = q[1], qy = q[2], qz = q[3];
  const double tx = 2.0 * (qy * z - qz * y), ty = 2.0 * (qz * x - qx * z),
               tz = 2.0 * (qx * y - qy * x);
  return {x + w * tx + (qy * tz - qz * ty), y + w * ty + (qz * tx - qx * tz),
          z + w * tz + (qx * ty - qy * tx)};
}

} // namespace

std::vector<spawn_point> road_spawn_points(const cvc::world::world_geometry &world,
                                           const nav::world_grid &grid, int n, double spacing,
                                           double clearance) {
  std::vector<spawn_point> out;
  if (n <= 0 || !(spacing > 0.0))
    return out;
  std::uint16_t asphalt = 0;
  const bool has_asphalt = world.surfaces().find("asphalt", asphalt);
  const std::vector<float> sdf = nav::signed_distance(grid);
  // One candidate per spacing-square bin: the triangle centre nearest the bin's centre, heading
  // along its longest edge (a road's quads run along it).
  struct pick {
    double d2 = 1e300, x = 0.0, y = 0.0, heading = 0.0;
  };
  std::map<std::pair<long, long>, pick> bins;
  for (const cvc::world::geometry_item &item : world.items()) {
    if (!item.geometry || !(item.name == "roads" || (has_asphalt && item.surface == asphalt)))
      continue;
    const auto &pts = item.geometry->const_points();
    for (const auto &t : item.geometry->const_tris()) {
      std::array<std::array<double, 3>, 3> v;
      for (int k = 0; k < 3; ++k) {
        const auto &p = pts[static_cast<std::size_t>(t[k])];
        v[k] = rotate(item.rotation, p[0], p[1], p[2]);
        for (int c = 0; c < 3; ++c)
          v[k][c] += item.position[c];
      }
      const double cx = (v[0][0] + v[1][0] + v[2][0]) / 3.0;
      const double cy = (v[0][1] + v[1][1] + v[2][1]) / 3.0;
      if (grid.occupied(cx, cy))
        continue;
      int r = 0, c = 0;
      grid.sample_of(cx, cy, r, c);
      if (sdf[static_cast<std::size_t>(r) * grid.cols + c] < clearance)
        continue;
      const std::pair<long, long> b{static_cast<long>(std::floor(cx / spacing)),
                                    static_cast<long>(std::floor(cy / spacing))};
      const double bx = (b.first + 0.5) * spacing, by = (b.second + 0.5) * spacing;
      const double d2 = (cx - bx) * (cx - bx) + (cy - by) * (cy - by);
      pick &best = bins[b];
      if (d2 >= best.d2)
        continue;
      double longest = -1.0, heading = 0.0;
      for (int k = 0; k < 3; ++k) {
        const auto &a = v[static_cast<std::size_t>(k)];
        const auto &e = v[static_cast<std::size_t>((k + 1) % 3)];
        const double l = std::hypot(e[0] - a[0], e[1] - a[1]);
        if (l > longest) {
          longest = l;
          heading = std::atan2(e[1] - a[1], e[0] - a[0]);
        }
      }
      if (heading < 0.0) // a road runs both ways: keep the heading in [0, pi)
        heading += kPi;
      if (heading >= kPi)
        heading -= kPi;
      best = {d2, cx, cy, heading};
    }
  }
  // The bins in a fixed order, then every k-th so the n spread over the whole network.
  std::vector<pick> all;
  for (const auto &[b, p] : bins)
    all.push_back(p);
  const std::size_t step = std::max<std::size_t>(1, all.size() / static_cast<std::size_t>(n));
  for (std::size_t i = 0; i < all.size() && static_cast<int>(out.size()) < n; i += step) {
    spawn_point s;
    s.x = all[i].x;
    s.y = all[i].y;
    s.heading = all[i].heading * 180.0 / kPi;
    out.push_back(s);
  }
  return out;
}

std::vector<spawn_point> water_spawn_points(const nav::world_grid &grid, int n, double spacing,
                                            double clearance) {
  std::vector<spawn_point> out;
  if (n <= 0 || !(spacing > 0.0) || grid.rows < 3 || grid.cols < 3)
    return out;
  const std::vector<float> sdf = nav::signed_distance(grid);
  const std::size_t cells = static_cast<std::size_t>(grid.rows) * grid.cols;
  const double h = grid.cell();
  const auto ok = [&](std::size_t i) { return !grid.occupancy[i] && sdf[i] >= clearance; };
  // Each body of water with room for a boat: the 4-connected runs of the samples it may start on.
  std::vector<int> body(cells, -1);
  int bodies = 0;
  for (std::size_t s = 0; s < cells; ++s) {
    if (body[s] >= 0 || !ok(s))
      continue;
    std::deque<std::size_t> q{s};
    body[s] = bodies;
    while (!q.empty()) {
      const std::size_t i = q.front();
      q.pop_front();
      const int r = static_cast<int>(i / grid.cols), c = static_cast<int>(i % grid.cols);
      const int nr[4] = {r - 1, r + 1, r, r}, nc[4] = {c, c, c - 1, c + 1};
      for (int k = 0; k < 4; ++k) {
        if (nr[k] < 0 || nr[k] >= grid.rows || nc[k] < 0 || nc[k] >= grid.cols)
          continue;
        const std::size_t j = static_cast<std::size_t>(nr[k]) * grid.cols + nc[k];
        if (body[j] < 0 && ok(j)) {
          body[j] = bodies;
          q.push_back(j);
        }
      }
    }
    ++bodies;
  }
  // One candidate per body and spacing-square bin: the sample nearest the bin's centre, heading
  // along the shore (across the distance field's gradient).
  struct pick {
    double d2 = 1e300, x = 0.0, y = 0.0, heading = 0.0;
  };
  std::vector<std::map<std::pair<long, long>, pick>> bins(static_cast<std::size_t>(bodies));
  for (std::size_t i = 0; i < cells; ++i) {
    if (body[i] < 0)
      continue;
    const int r = static_cast<int>(i / grid.cols), c = static_cast<int>(i % grid.cols);
    const double x = grid.min_x + c * h, y = grid.min_y + r * h;
    const std::pair<long, long> b{static_cast<long>(std::floor(x / spacing)),
                                  static_cast<long>(std::floor(y / spacing))};
    const double bx = (b.first + 0.5) * spacing, by = (b.second + 0.5) * spacing;
    const double d2 = (x - bx) * (x - bx) + (y - by) * (y - by);
    pick &best = bins[static_cast<std::size_t>(body[i])][b];
    if (d2 >= best.d2)
      continue;
    const auto at = [&](int rr, int cc) {
      rr = std::clamp(rr, 0, grid.rows - 1);
      cc = std::clamp(cc, 0, grid.cols - 1);
      return static_cast<double>(sdf[static_cast<std::size_t>(rr) * grid.cols + cc]);
    };
    const double gx = at(r, c + 1) - at(r, c - 1), gy = at(r + 1, c) - at(r - 1, c);
    double heading = gx == 0.0 && gy == 0.0 ? 0.0 : std::atan2(gx, -gy); // gradient turned 90 deg
    if (heading < 0.0)
      heading += 2.0 * kPi;
    best = {d2, x, y, heading};
  }
  // The bodies in turn, each one's bins in a fixed order spread over it.
  std::vector<std::vector<pick>> each(static_cast<std::size_t>(bodies));
  for (int k = 0; k < bodies; ++k)
    for (const auto &[b, p] : bins[static_cast<std::size_t>(k)])
      each[static_cast<std::size_t>(k)].push_back(p);
  std::vector<std::size_t> used(static_cast<std::size_t>(bodies), 0);
  const std::size_t share =
      std::max<std::size_t>(1, static_cast<std::size_t>(n) / std::max(1, bodies));
  for (bool more = true; more && static_cast<int>(out.size()) < n;) {
    more = false;
    for (int k = 0; k < bodies && static_cast<int>(out.size()) < n; ++k) {
      const std::vector<pick> &v = each[static_cast<std::size_t>(k)];
      const std::size_t step = std::max<std::size_t>(1, v.size() / share);
      const std::size_t i = used[static_cast<std::size_t>(k)] * step;
      if (i >= v.size())
        continue;
      ++used[static_cast<std::size_t>(k)];
      more = true;
      spawn_point s;
      s.x = v[i].x;
      s.y = v[i].y;
      s.heading = v[i].heading * 180.0 / kPi;
      out.push_back(s);
    }
  }
  return out;
}

bool WorldSpawns::start_ground::free_at(double x, double y) const {
  if (!grid)
    return true; // nothing to check against yet
  if (grid->occupied(x, y))
    return false;
  int r = 0, c = 0;
  grid->sample_of(x, y, r, c);
  return !sdf || sdf->empty() || (*sdf)[static_cast<std::size_t>(r) * grid->cols + c] >= clearance;
}

bool WorldSpawns::start_ground::any() const {
  if (!grid || !sdf)
    return false;
  for (std::size_t i = 0; i < sdf->size(); ++i)
    if (!grid->occupancy[i] && (*sdf)[i] >= clearance)
      return true;
  return false;
}

struct WorldSpawns::impl {
  cvc::app *app = nullptr;
  std::string key;
  spawn_rules rules;
  struct point {
    int id = 0;
    spawn_point p;
  };
  std::vector<point> all; // every world's
  int next_id = 1;
  std::string world_on;
  std::shared_ptr<const nav::world_grid> grid;
  std::shared_ptr<const cvc::world::world_geometry> geometry;
  std::vector<float> sdf;
  std::map<std::string, spawn_need> needs; // by archetype
  // The medium grids made so far for the world on, by medium and depth.
  struct made {
    std::shared_ptr<const nav::world_grid> grid;
    std::shared_ptr<const std::vector<float>> sdf;
  };
  mutable std::map<std::pair<int, double>, made> grounds;
  std::vector<spawn_point> suggested;
  std::map<std::string, std::uint64_t> cursor; // round_robin, by scope group
  std::uint64_t draws = 0;
  std::uint64_t rev = 0, published_rev = ~std::uint64_t(0);
  std::string status;
  std::set<std::string> published;
  std::string seen_pick, seen_suggest, seen_accept, seen_clear;

  cvc::state *root() const { return app ? &cvc::state::instance(*app) : nullptr; }
  std::string get(const std::string &leaf) const {
    cvc::state *r = root();
    if (!r || key.empty())
      return std::string();
    cvc::state *k = r->findDescendant(key + cvc::state::SEPARATOR + leaf);
    return k ? k->value() : std::string();
  }
  void put(const std::string &leaf, const std::string &value) {
    if (cvc::state *r = root(); r && !key.empty())
      (*r)(key + "." + leaf).value(value);
  }
  void seed(const std::string &leaf, const std::string &value) {
    cvc::state *r = root();
    if (!r || key.empty())
      return;
    cvc::state *k = r->findDescendant(key + cvc::state::SEPARATOR + leaf);
    if (!k || k->value().empty())
      (*r)(key + "." + leaf).value(value);
  }
  double num_key(const std::string &leaf, double def) const {
    try {
      const std::string v = get(leaf);
      return v.empty() ? def : std::stod(v);
    } catch (const std::exception &) {
      return def;
    }
  }

  bool on_world(const spawn_point &p) const { return p.world.empty() || p.world == world_on; }
  spawn_need need_of(const std::string &archetype) const {
    const auto it = needs.find(archetype);
    return it == needs.end() ? spawn_need{} : it->second;
  }
  start_ground ground_for(const spawn_need &n) const {
    start_ground g;
    const bool water = n.medium == cvc::nav::plant_medium::water;
    g.clearance = water ? std::max(kClearance, n.half_length + rules.shore_clearance) : kClearance;
    if (!grid)
      return g;
    const double depth = water ? n.draft + rules.keel_clearance : rules.ford_depth;
    made &m = grounds[{water ? 1 : 0, depth}];
    if (!m.grid) {
      auto mg = std::make_shared<nav::world_grid>(nav::medium_grid(*grid, n.medium, depth));
      m.sdf = std::make_shared<const std::vector<float>>(nav::signed_distance(*mg));
      m.grid = std::move(mg);
    }
    g.grid = m.grid;
    g.sdf = m.sdf;
    return g;
  }
  // A point's archetype's ground is free there; an untagged point's, any archetype's: what it is
  // free for, as `medium` (land, water, both or "").
  bool free_for(const spawn_point &p, std::string *medium = nullptr) const {
    if (!p.archetype.empty()) {
      const spawn_need n = need_of(p.archetype);
      const bool ok = ground_for(n).free_at(p.x, p.y);
      if (medium)
        *medium = !ok ? "" : n.medium == cvc::nav::plant_medium::water ? "water" : "land";
      return ok;
    }
    bool land = ground_for(spawn_need{}).free_at(p.x, p.y), water = false;
    for (const auto &[id, n] : needs)
      if (!water && n.medium == cvc::nav::plant_medium::water)
        water = ground_for(n).free_at(p.x, p.y);
    if (medium)
      *medium = land && water ? "both" : land ? "land" : water ? "water" : "";
    return land || water;
  }
  double ground(double x, double y) const {
    if (!grid || grid->height.empty())
      return 0.0;
    int r = 0, c = 0;
    grid->sample_of(x, y, r, c);
    return grid->height[static_cast<std::size_t>(r) * grid->cols + c];
  }
  std::string policy() const {
    const std::string p = get("policy");
    return spawn_policy_known(p) ? p : rules.policy;
  }
  std::string scope() const {
    const std::string s = get("scope");
    return spawn_scope_known(s) ? s : rules.scope;
  }
};

WorldSpawns::WorldSpawns(cvc::app *app, spawn_rules rules, std::string key)
    : _impl(std::make_unique<impl>()) {
  impl &d = *_impl;
  d.app = app;
  d.key = std::move(key);
  d.rules = std::move(rules);
  for (const spawn_point &p : d.rules.points)
    d.all.push_back({d.next_id++, p});
  d.seed("policy", d.rules.policy);
  d.seed("scope", d.rules.scope);
  d.seed("tool", "add");
  d.seed("heading", "0");
  d.seed("tag.team", "-1");
  d.seed("suggest.count", "12");
  d.seed("suggest.spacing", "30");
  for (const char *req : {"suggest.request", "accept.request", "clear.request"})
    d.seed(req, "0");
  d.seen_pick = d.get("pick.seq");
  d.seen_suggest = d.get("suggest.request");
  d.seen_accept = d.get("accept.request");
  d.seen_clear = d.get("clear.request");
  d.status = "waiting for the world's grid";
  publish();
}

WorldSpawns::~WorldSpawns() = default;

void WorldSpawns::set_world(const std::string &name, std::shared_ptr<const nav::world_grid> grid,
                            std::shared_ptr<const cvc::world::world_geometry> world) {
  impl &d = *_impl;
  d.world_on = name;
  d.grid = std::move(grid);
  d.geometry = std::move(world);
  d.sdf = d.grid ? nav::signed_distance(*d.grid) : std::vector<float>();
  d.grounds.clear();
  d.suggested.clear();
  d.status = d.grid ? "ready" : "waiting for the world's grid";
  ++d.rev;
  publish();
}

void WorldSpawns::set_needs(std::map<std::string, spawn_need> needs) {
  impl &d = *_impl;
  d.needs = std::move(needs);
  d.grounds.clear();
  ++d.rev;
  publish();
}

spawn_need WorldSpawns::need_of(const std::string &archetype) const {
  return _impl->need_of(archetype);
}

WorldSpawns::start_ground WorldSpawns::ground_for(const std::string &archetype) const {
  return _impl->ground_for(_impl->need_of(archetype));
}

WorldSpawns::start_ground WorldSpawns::ground_for(const spawn_need &need) const {
  return _impl->ground_for(need);
}

bool WorldSpawns::choose(int team, const std::string &archetype,
                         const std::function<bool(double, double, double)> &crowded, double &x,
                         double &y, double &heading_rad) {
  impl &d = *_impl;
  // Its medium's ground: an untagged point serves it only where that is free.
  const start_ground ground = d.ground_for(d.need_of(archetype));
  std::vector<const impl::point *> fit;
  for (const impl::point &p : d.all)
    if (d.on_world(p.p) && ground.free_at(p.p.x, p.p.y) && (p.p.team < 0 || p.p.team == team) &&
        (p.p.archetype.empty() || p.p.archetype == archetype))
      fit.push_back(&p);
  if (fit.empty())
    return false;
  const std::string policy = d.policy(), scope = d.scope();
  const std::string group = scope == "team"        ? "team " + std::to_string(team)
                            : scope == "archetype" ? "archetype " + archetype
                                                   : std::string("all");
  const impl::point *chosen = fit.front();
  if (policy == "random") {
    chosen = fit[mix(0x5eed ^ ++d.draws ^ std::hash<std::string>{}(group)) % fit.size()];
  } else if (policy == "round_robin") {
    chosen = fit[d.cursor[group]++ % fit.size()];
  } else if (policy == "nearest_free") {
    chosen = nullptr;
    for (const impl::point *p : fit)
      if (!crowded || !crowded(p->p.x, p->p.y, kNear)) {
        chosen = p;
        break;
      }
    if (!chosen)
      chosen = fit.front();
  }
  // A vehicle on the point: aside along the heading's normal, a vehicle width at a time.
  const double h = chosen->p.heading * kPi / 180.0;
  const double nx = -std::sin(h), ny = std::cos(h);
  x = chosen->p.x;
  y = chosen->p.y;
  for (int k = 1; k <= 8 && crowded && crowded(x, y, kNear * 0.75); ++k) {
    const double step = (k % 2 ? 1.0 : -1.0) * ((k + 1) / 2) * kNear;
    const double tx = chosen->p.x + nx * step, ty = chosen->p.y + ny * step;
    if (ground.free_at(tx, ty)) {
      x = tx;
      y = ty;
    }
  }
  heading_rad = h;
  return true;
}

int WorldSpawns::add(spawn_point p, std::string *why) {
  impl &d = *_impl;
  const auto refuse = [&](const std::string &w) {
    if (why)
      *why = w;
    d.status = w;
    return 0;
  };
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.heading))
    return refuse("a spawn point must be finite");
  if (d.grid &&
      (p.x < d.grid->min_x || p.x > d.grid->max_x || p.y < d.grid->min_y || p.y > d.grid->max_y))
    return refuse("(" + fixed(p.x, 1) + ", " + fixed(p.y, 1) + ") is off the world");
  if (!d.free_for(p))
    return refuse("(" + fixed(p.x, 1) + ", " + fixed(p.y, 1) +
                  ") is blocked, too near something, " +
                  (p.archetype.empty() ? std::string("or on no vehicle's ground (land, or water "
                                                     "deep and open enough for a boat)")
                                       : "or not on " + p.archetype + "'s ground"));
  if (const int close = nearest(p.x, p.y, kNear))
    return refuse("point " + std::to_string(close) + " is within " + num(kNear) + " m");
  if (p.world.empty())
    p.world = d.world_on;
  const int id = d.next_id++;
  d.all.push_back({id, std::move(p)});
  d.status = "added point " + std::to_string(id);
  ++d.rev;
  return id;
}

bool WorldSpawns::remove(int id) {
  impl &d = *_impl;
  for (auto it = d.all.begin(); it != d.all.end(); ++it)
    if (it->id == id) {
      d.all.erase(it);
      d.status = "removed point " + std::to_string(id);
      ++d.rev;
      return true;
    }
  return false;
}

bool WorldSpawns::tag(int id, int team, const std::string &archetype) {
  impl &d = *_impl;
  for (impl::point &p : d.all)
    if (p.id == id) {
      p.p.team = team;
      p.p.archetype = archetype;
      d.status = "tagged point " + std::to_string(id);
      ++d.rev;
      return true;
    }
  return false;
}

void WorldSpawns::clear() {
  impl &d = *_impl;
  d.all.erase(std::remove_if(d.all.begin(), d.all.end(),
                             [&](const impl::point &p) { return d.on_world(p.p); }),
              d.all.end());
  d.status = "cleared";
  ++d.rev;
}

int WorldSpawns::nearest(double x, double y, double r) const {
  int best = 0;
  double bd = r;
  for (const impl::point &p : _impl->all)
    if (_impl->on_world(p.p)) {
      const double dd = std::hypot(p.p.x - x, p.p.y - y);
      if (dd <= bd) {
        bd = dd;
        best = p.id;
      }
    }
  return best;
}

std::vector<spawn_point> WorldSpawns::suggest(int n, double spacing) {
  impl &d = *_impl;
  d.suggested.clear();
  // The tagged archetype's medium: a boat's points over the water, a land vehicle's on the roads
  // (out of water deeper than a ford).
  const std::string archetype = d.get("tag.archetype");
  const spawn_need need = d.need_of(archetype);
  const start_ground ground = d.ground_for(need);
  const bool water = need.medium == cvc::nav::plant_medium::water;
  if (ground.grid) {
    const std::vector<spawn_point> found =
        water        ? water_spawn_points(*ground.grid, n, spacing, ground.clearance)
        : d.geometry ? road_spawn_points(*d.geometry, *ground.grid, n, spacing, ground.clearance)
                     : std::vector<spawn_point>();
    for (spawn_point p : found)
      if (!nearest(p.x, p.y, kNear))
        d.suggested.push_back(std::move(p));
  }
  if (water)
    d.status = d.suggested.empty() ? "no water deep and open enough for " + archetype
                                   : std::to_string(d.suggested.size()) +
                                         " points suggested over the water for " + archetype;
  else
    d.status =
        d.suggested.empty()
            ? std::string(d.geometry ? "no roads with room for a point" : "this world has no roads")
            : std::to_string(d.suggested.size()) + " points suggested along the roads";
  ++d.rev;
  return d.suggested;
}

int WorldSpawns::accept() {
  impl &d = *_impl;
  int added = 0;
  const std::vector<spawn_point> take = std::move(d.suggested);
  d.suggested.clear();
  for (spawn_point p : take) {
    p.team = static_cast<int>(d.num_key("tag.team", -1));
    p.archetype = d.get("tag.archetype");
    added += add(std::move(p)) != 0;
  }
  d.status = "added " + std::to_string(added) + " suggested points";
  ++d.rev;
  return added;
}

void WorldSpawns::poll() {
  impl &d = *_impl;
  if (!d.root() || d.key.empty())
    return;
  if (const std::string s = d.get("pick.seq"); s != d.seen_pick) {
    d.seen_pick = s;
    if (d.get("pick.hit") == "1" && d.get("pick.phase") != "drag" && d.get("pick.phase") != "up") {
      const double x = d.num_key("pick.x", 0.0), y = d.num_key("pick.y", 0.0);
      const std::string tool = d.get("tool");
      const int team = static_cast<int>(d.num_key("tag.team", -1));
      const std::string archetype = d.get("tag.archetype");
      if (tool == "remove") {
        if (const int id = nearest(x, y, kNear))
          remove(id);
        else
          d.status = "no point within " + num(kNear) + " m";
      } else if (tool == "tag") {
        if (const int id = nearest(x, y, kNear))
          tag(id, team, archetype);
        else
          d.status = "no point within " + num(kNear) + " m";
      } else {
        spawn_point p;
        p.x = x;
        p.y = y;
        p.heading = d.num_key("heading", 0.0);
        p.team = team;
        p.archetype = archetype;
        add(std::move(p));
      }
    }
  }
  if (const std::string s = d.get("suggest.request"); s != d.seen_suggest) {
    d.seen_suggest = s;
    suggest(static_cast<int>(d.num_key("suggest.count", 12)), d.num_key("suggest.spacing", 30));
  }
  if (const std::string s = d.get("accept.request"); s != d.seen_accept) {
    d.seen_accept = s;
    accept();
  }
  if (const std::string s = d.get("clear.request"); s != d.seen_clear) {
    d.seen_clear = s;
    clear();
  }
  publish();
}

void WorldSpawns::publish() {
  impl &d = *_impl;
  cvc::state *r = d.root();
  if (!r || d.key.empty())
    return;
  if (d.published_rev == d.rev) {
    d.put("status", d.status);
    return; // nothing new
  }
  d.published_rev = d.rev;
  std::set<std::string> now;
  std::string ids, summary;
  for (const entry &e : points()) {
    const std::string id = std::to_string(e.id);
    now.insert(id);
    ids += (ids.empty() ? "" : ",") + id;
    const std::string at = "points." + id + ".";
    d.put(at + "x", num(e.point.x));
    d.put(at + "y", num(e.point.y));
    d.put(at + "z", num(e.z));
    d.put(at + "heading", num(e.point.heading));
    d.put(at + "team", std::to_string(e.point.team));
    d.put(at + "archetype", e.point.archetype);
    d.put(at + "world", e.point.world);
    d.put(at + "free", e.free ? "1" : "0");
    d.put(at + "medium", e.medium);
    summary +=
        (summary.empty() ? "" : "\n") + id + "  " + fixed(e.point.x, 1) + ", " +
        fixed(e.point.y, 1) + "  " + fixed(e.point.heading, 0) + " deg  " +
        (e.point.team < 0 ? std::string("any team") : "team " + std::to_string(e.point.team)) +
        "  " + (e.point.archetype.empty() ? std::string("any vehicle") : e.point.archetype) +
        (e.free ? "" : "  BLOCKED");
  }
  if (cvc::state *t = r->findDescendant(d.key + cvc::state::SEPARATOR + "points"))
    for (const std::string &id : d.published)
      if (!now.count(id))
        t->removeChild(id);
  d.published = now;
  std::string sugg;
  for (std::size_t i = 0; i < d.suggested.size(); ++i) {
    const spawn_point &p = d.suggested[i];
    d.put("suggestions." + std::to_string(i) + ".x", num(p.x));
    d.put("suggestions." + std::to_string(i) + ".y", num(p.y));
    d.put("suggestions." + std::to_string(i) + ".heading", num(p.heading));
    sugg += (sugg.empty() ? "" : "\n") + fixed(p.x, 1) + ", " + fixed(p.y, 1) + "  " +
            fixed(p.heading, 0) + " deg";
  }
  d.put("ids", ids);
  d.put("count", std::to_string(now.size()));
  d.put("summary", summary);
  d.put("suggestions.count", std::to_string(d.suggested.size()));
  d.put("suggestions.summary", sugg);
  d.put("world", d.world_on);
  d.put("status", d.status);
  d.put("rev", std::to_string(d.rev)); // last: a watcher of rev sees the rest written
}

std::vector<WorldSpawns::entry> WorldSpawns::points() const {
  std::vector<entry> out;
  for (const impl::point &p : _impl->all)
    if (_impl->on_world(p.p)) {
      entry e;
      e.id = p.id;
      e.point = p.p;
      e.z = _impl->ground(p.p.x, p.p.y);
      e.free = _impl->free_for(p.p, &e.medium);
      out.push_back(std::move(e));
    }
  return out;
}

const std::vector<spawn_point> &WorldSpawns::suggestions() const { return _impl->suggested; }
double WorldSpawns::ground(double x, double y) const { return _impl->ground(x, y); }
std::string WorldSpawns::policy() const { return _impl->policy(); }
std::string WorldSpawns::scope() const { return _impl->scope(); }
std::uint64_t WorldSpawns::revision() const { return _impl->rev; }
const std::string &WorldSpawns::status() const { return _impl->status; }

} // namespace ariadne
} // namespace cvc
