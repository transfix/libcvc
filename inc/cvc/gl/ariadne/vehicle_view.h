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

#ifndef __CVC_GL_ARIADNE_VEHICLE_VIEW_H__
#define __CVC_GL_ARIADNE_VEHICLE_VIEW_H__

#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/world_rules.h> // vehicle_look
#include <string>
#include <vector>

namespace cvc {
namespace gl {
namespace ariadne {

struct RealizedScene;
struct vehicle_model_cache; // the vehicle models a realized scene has fitted

// ----------------
// A world's vehicles, drawn (ARIADNE_WORLDS.md §14, L2)
// ----------------
// The view half of the world runtime's vehicles (cvc::ariadne::WorldVehicles): each entity drawn
// on its chassis body, posed between the last two physics steps as the scene's other moving
// bodies are; and the `vehicle_preview` scene node, which draws the archetype a vehicle_selector
// has picked, alone on a studio set, turning.

// A vehicle's size, as the catalog publishes it (<prefix>.vehicles.archetypes.<id>.*).
struct vehicle_shape {
  double length = 4.84, width = 2.18, height = 0.9; // the chassis box (m)
  double wheelbase = 3.3, track = 1.82, wheel_radius = 0.45;
};

// The scene node a vehicle is drawn as, about its chassis box's centre: a group `id` holding its
// look. The box look is the chassis box in `color`, a cabin in `color2` and four dark wheels; a
// file look is the mesh, placed by the look's scale, yaw and z.
cvc::ariadne::SceneNode vehicle_scene_node(const std::string &id,
                                           const cvc::ariadne::vehicle_look &look,
                                           const vehicle_shape &shape, const float color[3],
                                           const float color2[3]);

// Each frame after the world steps (tick_scene): when the vehicles' revision has moved, the
// entities' drawings are built again and bound to their chassis bodies.
void sync_vehicle_markers(RealizedScene &realized, std::vector<std::string> *warnings);
// Pose the vehicles that have no chassis in the scene's physics world (nav's bicycle, the
// dynamic models in a world of their own) from where the runtime says they are, in the physics
// binding's frame. Each frame, after the bodies are posed.
void pose_vehicle_markers(RealizedScene &realized);

// Each frame after the world steps: when the spawn points' revision has moved, their markers are
// built again: a pad in its team's colour with a pointer along its heading (white for any team),
// and a grey one for each suggestion, all tagged `spawns` (a view's show: / hide: picks them).
void sync_spawn_markers(RealizedScene &realized, std::vector<std::string> *warnings);

// `type: vehicle_preview` (props: `vehicles:`, the catalog's key root, default vehicles;
// `spin:`, degrees a second, default 20): the archetype `<vehicles>.selected` names, in the
// spawn keys' colours, at the node's position. Registered on `app`; idempotent.
void register_vehicle_preview_node(cvc::app &app);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // __CVC_GL_ARIADNE_VEHICLE_VIEW_H__
