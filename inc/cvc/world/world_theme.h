/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// world_theme.h — which model assets a generated world builds and plants, and where.
//
// A theme is data: a list of building archetypes and a list of tree archetypes, each naming a
// model asset (cvc/world/model_assets.h) with a weight and the conditions it suits. The generated
// world (generated_world_params::theme) asks its theme, lot by lot and tree by tree:
//
//   * a lot (cvc/world/sites.h) takes a building archetype whose urbanity band holds the lot's and
//     whose footprint fits it (at a scale in [min_scale, max_scale], next to its neighbour on the
//     same street side when `span` allows and one lot is too narrow), drawn by weight; with none,
//     or when the lot's draw is over building_share, it keeps the city's procedural massing;
//   * a tree the scatter placed becomes a tree archetype, drawn by weight among those whose biome
//     rule holds at its spot (height above the water, slope, distance to the coast), when its draw
//     is under tree_share; otherwise it stays the L-system tree.
//
// Every draw is keyed on the spot (lsys::cell_id), so the same world params, theme and catalog
// make the same world; adding an archetype changes only the draws it now wins. An archetype whose
// asset is not in the catalog (its package is not installed) is left out, as if the theme did not
// name it.
//
// Themes are the app's, by name, like the terrain and city generators: the built-ins are below
// (world_theme_names), and register_world_theme adds or replaces one; world_theme_from_json reads
// one from a document, so a scene can carry its own.

#ifndef CVC_WORLD_WORLD_THEME_H
#define CVC_WORLD_WORLD_THEME_H

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace world {

class model_catalog;

// Where a tree archetype grows. Every bound holds by default.
struct biome_rule {
  // The ground's height over the water level (metres).
  double min_height_m = -std::numeric_limits<double>::infinity();
  double max_height_m = std::numeric_limits<double>::infinity();
  double max_slope_deg = 90.0;
  // The distance to the nearest water (the sea, a lake or a pond), metres: a palm's coast, an
  // oak's inland. Measured on the world's material raster.
  double min_water_m = 0.0;
  double max_water_m = std::numeric_limits<double>::infinity();
};

struct building_archetype_rule {
  std::string asset;
  double weight = 1.0;
  double min_urbanity = 0.0, max_urbanity = 1.0;
  double min_scale = 0.9, max_scale = 1.1; // the scale drawn per building, within what fits
  int span = 1; // how many neighbouring lots on one street side it may take (1: its own)
};

struct tree_archetype_rule {
  std::string asset;
  double weight = 1.0;
  double min_scale = 0.8, max_scale = 1.2;
  biome_rule biome;
};

struct world_theme {
  std::string name;
  std::string description;
  std::vector<building_archetype_rule> buildings;
  double building_share = 0.7; // the share of lots that try an archetype before the massing
  std::vector<tree_archetype_rule> trees;
  double tree_share = 0.6; // the share of trees that try an archetype before the L-system tree
};

// The built-in themes:
//   "temperate_town": the brick corner shop and brownstones along the streets and the office in
//                     the cores; oaks and young oaks, none on the beach.
//   "coastal_town":   the same streets; palms along the coast (within 60 m of water and below
//                     10 m over it), oaks inland.
//   "tropical":       palms everywhere but steep ground, the odd oak on the hills; a low-rise
//                     town (corner shops and brownstones), the office in the cores.
const std::vector<world_theme> &builtin_world_themes();

// The app's themes. register_world_theme adds a theme or replaces the one of its name.
void register_world_theme(cvc::app &app, const world_theme &t);
std::vector<std::string> world_theme_names(cvc::app &app); // built-ins first, then registered
// The theme of that name; false when there is none.
bool find_world_theme(cvc::app &app, const std::string &name, world_theme &out);

// A theme as JSON, and back. Fields a text leaves out keep their defaults; an archetype without
// an asset, a negative weight, an inverted band or a share outside [0, 1] throws
// std::runtime_error naming `what`.
std::string world_theme_json(const world_theme &t);
world_theme world_theme_from_json(const std::string &text, const std::string &what = "world theme");

// ── the draws (what the generated world calls, exposed for tests and other generators) ──────────

// A lot as a building archetype sees it.
struct lot_view {
  double x = 0, y = 0; // the lot's centre
  double frontage = 0; // its width along the road (with any neighbours it may take)
  double depth = 0;    // its depth from the setback line
  double urbanity = 0;
  int lots = 1; // how many lots `frontage` spans
};
// The archetype a lot takes, as an index into theme.buildings, and its scale; -1 when the lot
// keeps the massing (its draw is over building_share, or nothing fits). An archetype fits when
// it is in the catalog, its band holds the lot's urbanity, it spans at least lot.lots lots, and
// its footprint (plus `margin` a side) fits the frontage and depth at some scale in
// [min_scale, max_scale]: the scale is then drawn from what fits.
struct building_pick {
  int archetype = -1;
  double scale = 1.0;
};
building_pick pick_building(const world_theme &theme, const model_catalog &catalog,
                            const lot_view &lot, std::uint64_t seed, double margin = 1.0);

// What a tree archetype sees at its spot.
struct tree_site {
  double x = 0, y = 0;
  double height_m = 0; // over the water level
  double slope_deg = 0;
  double water_m = std::numeric_limits<double>::infinity(); // to the nearest water
};
bool biome_holds(const biome_rule &b, const tree_site &s);
// The archetype a tree becomes (an index into theme.trees) and its scale and yaw (radians); -1
// when it stays the L-system tree.
struct tree_pick {
  int archetype = -1;
  double scale = 1.0;
  double yaw = 0.0;
};
tree_pick pick_tree(const world_theme &theme, const model_catalog &catalog, const tree_site &site,
                    std::uint64_t seed);

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_WORLD_THEME_H
