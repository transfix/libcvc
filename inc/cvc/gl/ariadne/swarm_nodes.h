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

// swarm_nodes.h — a swarm of navigating vehicles in a document's scene (docs/ARIADNE.md §5.4,
// "Swarms"): the `nav_swarm` node runs one (cvc::ariadne::SceneSwarm), and each `swarm_layer` node
// under it draws one part of it, a node like any other, with its own visible:, tags: and material:.
//
//   - node: swarm
//     type: nav_swarm
//     city: city           # a data: entry's name: the city bundle it crosses (the synthetic city
//                          # when it has none, or its load fails)
//     agents: 800          # belief, fog, seed, spawn, targets, reach, separate, sep_radius,
//     belief: grouped      # sep_gain, plant, policy, grid, sense_every (scene_swarm.h), each a
//     targets: 16          # key under keys: (swarm) that a panel drives
//     keys: swarm
//     children:
//       - { node: swarm_agents, type: swarm_layer, layer: agents, look: humvee, lod: true }
//       - { node: swarm_flags, type: swarm_layer, layer: flags, visible: show.flags }
//       - { node: swarm_dots, type: swarm_layer, layer: dots, tags: [minimap] }
//       - { node: chase, type: swarm_layer, layer: follow }   # what track.target names
//
// The layers:
//   agents   the vehicles: a vehicle model (look: humvee, the vehicle-humvee package's, or a mesh
//            file), else arrows; with lod: (on), four quadric-decimated rungs of the model picked
//            per vehicle by its size on screen (near_cap: the full-detail budget, 10), off-screen
//            vehicles culled. Without lod, white while wall-following and green once there.
//   flags    a pole and a flag in each vehicle's colour above it
//   goals    a beacon over each vehicle's goal, sunk once it is there
//   trails   the last `segments:` (12) moves of each vehicle, every `every:` (3) frames; `width:`
//            a number or a state key
//   path     the followed vehicle's trail, wider and yellow
//   belief   the walls the fleet has sensed, see-through boxes in the colours of the belief planes
//            that saw them, cycling where several did (`height:`; material: { opacity })
//   dots     a marker per vehicle over everything, for a map view
//   targets  a disc per rally point over everything, for a map view
//   city     the synthetic city's blocks and ground (its fog of war on the ground under fog); on a
//            real city, nothing
//   follow   an empty node at the followed vehicle, heading its way: a track camera's target
//
// Each frame (tick_scene) the swarm takes its keys, runs the steps due on the world clock and
// hands the layers its newest frame. The node also writes the frame rate it sees to
// `<keys>.stats.fps` twice a second, and with `<keys>.bench` 1 prints it on stderr every 120 frames
// with the time the swarm and its layers took (a benchmark that reads no window).
// register_swarm_node_types is part of register_cvcgl_extensions.

#ifndef CVC_GL_ARIADNE_SWARM_NODES_H
#define CVC_GL_ARIADNE_SWARM_NODES_H

namespace cvc {
class app;
namespace gl {
namespace ariadne {

// Register "nav_swarm" and "swarm_layer" on `app`. Idempotent.
void register_swarm_node_types(cvc::app &app);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_SWARM_NODES_H
