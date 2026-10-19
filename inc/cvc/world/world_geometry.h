/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::world::world_geometry — the world as data: its surface table and its
// geometry, one set of objects that the renderer, the physics layer and the
// navigator all read (docs/roadmap/WORLD-GEOMETRY-ROADMAP.md, G3).
//
//   cvc::world::world_geometry w;                     // the builtin surface table
//   w.add({"terrain", field});                        // a cvc::height_field
//   w.add({"buildings", mesh, w.surfaces().by_name("building_concrete")});
//   w.add_volume({"lake", lake});                     // a cvc::physics::field (G6)
//   cvc::gl::add_world_geometry(sg, w);               // the scene's nodes
//   cvc::physics::add_world_geometry(physics, w);     // static bodies and fields
//
// Items share their geometry: a consumer holds the same std::shared_ptr, never a
// copy, so a terrain edit is one edit. Frame: SI metres, z up, world coordinates.
//
// Materials: face_materials() and a height field's cell materials are ids in
// surfaces(); a face without one takes its item's `surface`. Each consumer turns a
// surface into its own terms (the renderer an albedo, physics a contact material,
// the navigator risk and hardness), so all of them read one table.
//
// Volumes: water, snow and wind as cvc::volume data (volume_item). The renderer
// draws the volume and the physics world pushes the bodies inside it; field.h
// says what the values mean for each kind.
//
// Not synchronized: build a world_geometry, then share it read-only.

#ifndef CVC_WORLD_WORLD_GEOMETRY_H
#define CVC_WORLD_WORLD_GEOMETRY_H

#include <array>
#include <cstdint>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/physics/field.h>
#include <cvc/world/surface.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace world {

// What an item is for. Every consumer takes the items with its role.
enum geometry_role : std::uint8_t {
  role_render = 1,   // drawn (cvc::gl::add_world_geometry)
  role_collide = 2,  // in the physics world: a static body, or a volume's field
                     // (cvc::physics::add_world_geometry)
  role_navigate = 4, // rasterized for the navigator
  role_all = 7,
};

struct geometry_item {
  std::string name;
  // A cvc::height_field (terrain) or a mesh.
  std::shared_ptr<const cvc::geometry> geometry;
  // The surface of every face that has no face material of its own.
  std::uint16_t surface = 0;
  // Placement of a mesh in the world (a height field is placed by its own grid).
  std::array<double, 3> position{{0.0, 0.0, 0.0}};
  std::array<double, 4> rotation{{1.0, 0.0, 0.0, 0.0}}; // unit quaternion: w, x, y, z
  std::uint8_t roles = role_all;

  // The geometry as a height field, or null for a mesh.
  std::shared_ptr<const cvc::height_field> height_field() const;
  bool has(geometry_role r) const { return (roles & r) != 0; }
};

// A region of water, snow or moving air: a cvc::physics::field, whose volume (and
// velocity volumes) are world-placed cvc::volume data. Not navigated yet, so its
// roles default to render and collide.
struct volume_item {
  std::string name;
  cvc::physics::field field;
  std::uint8_t roles = role_render | role_collide;

  bool has(geometry_role r) const { return (roles & r) != 0; }
};

// Axis-aligned bounds, min then max. Empty (min > max) when there is no geometry.
struct world_bounds {
  std::array<double, 3> min{{1.0, 1.0, 1.0}};
  std::array<double, 3> max{{-1.0, -1.0, -1.0}};
  bool empty() const { return min[0] > max[0]; }
};

class world_geometry {
public:
  // The surface table must outlive the world_geometry (the builtin ones are static).
  explicit world_geometry(const surface_registry &surfaces = surface_registry::builtin());

  const surface_registry &surfaces() const { return *_surfaces; }

  // Add an item. Throws std::invalid_argument for an empty name, a name already
  // used, a null geometry, a surface outside the table, a face material outside
  // the table, or a rotation that is not a unit quaternion.
  void add(geometry_item item);

  const std::vector<geometry_item> &items() const { return _items; }
  // The item called `name`, or null.
  const geometry_item *find(const std::string &name) const;

  // Add a volume. Names are shared with the geometry items: one name, one thing.
  // Throws std::invalid_argument for an empty or used name or a field without a
  // volume (the physics world checks the rest when it takes the field).
  void add_volume(volume_item item);
  const std::vector<volume_item> &volumes() const { return _volumes; }
  // The volume called `name`, or null.
  const volume_item *find_volume(const std::string &name) const;
  // The first height field, or null: the terrain most worlds have one of.
  std::shared_ptr<const cvc::height_field> terrain() const;

  // The bounds of every item and volume, placement applied (a rotated item's box
  // is the box of its rotated vertices; a volume's is its bounding box).
  world_bounds bounds() const;
  // Every surface id some face of some item uses, ascending.
  std::vector<std::uint16_t> surfaces_used() const;

private:
  const surface_registry *_surfaces;
  std::vector<geometry_item> _items;
  std::vector<volume_item> _volumes;
};

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_WORLD_GEOMETRY_H
