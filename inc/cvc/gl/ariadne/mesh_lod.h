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

// mesh_lod.h: a vehicle model's coarser rungs (swarm_nodes.h's agents, entity_view.h's bodies).

#ifndef CVC_GL_ARIADNE_MESH_LOD_H
#define CVC_GL_ARIADNE_MESH_LOD_H

#include <cstdint>
#include <map>
#include <utility>
#include <vector>

namespace cvc {
namespace gl {
namespace ariadne {

// A triangle mesh (`verts` [3V], `tris` [3T]) decimated to about (1 - reduction) of its triangles
// by VTK's quadric error metric, which keeps a silhouette, into `ov` and `ot`: welded first (a
// model's mesh is split at its texture seams, and the decimator collapses edges only within a
// connected surface), and only the points its triangles use. False when it collapsed (fewer than 4
// triangles left).
bool decimate_mesh(const std::vector<double> &verts, const std::vector<std::uint32_t> &tris,
                   double reduction, std::vector<double> &ov, std::vector<std::uint32_t> &ot);

// A mesh whose points each have a colour role (`role` [V], `own` [3V]: vehicle_view.h's
// vehicle_look_mesh) decimated role by role, so each part keeps its colour (a wheeled look's
// tyres their dark): each role's triangles on their own (decimate_mesh), a role's that collapse
// kept as they are, into `ov`, `ot`, `orole` and `oown` (a decimated part's points take its first
// point's own colour). False, the outputs untouched, when no role got coarser.
//
// `coarsest`, when given, carries each role's coarsest decimation (its points and triangles) from
// one rung to the next coarser one: a role that decimates is stored there, and a role that
// collapses takes what is stored instead of being kept as it is (a small mesh, a boat's hull, at
// the coarsest rung), so a coarser rung is never finer than the one before it.
using role_decimations =
    std::map<unsigned char, std::pair<std::vector<double>, std::vector<std::uint32_t>>>;
bool decimate_mesh_by_role(const std::vector<double> &verts, const std::vector<std::uint32_t> &tris,
                           const std::vector<unsigned char> &role,
                           const std::vector<unsigned char> &own, double reduction,
                           std::vector<double> &ov, std::vector<std::uint32_t> &ot,
                           std::vector<unsigned char> &orole, std::vector<unsigned char> &oown,
                           role_decimations *coarsest = nullptr);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_MESH_LOD_H
