/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_VOLUME_CODEC_H__
#define __CVC_STATE_VOLUME_CODEC_H__

#include <cvc/namespace.h>
#include <cvc/state_codec_registry.h>
#include <string>

namespace CVC_NAMESPACE {

// ----------------
// Volume / Geometry codec registration
// ----------------
// Registers encode/decode pairs for cvc::volume and cvc::geometry
// into a state_codec_registry so the distributed-state layer can
// serialize them into blob payloads.
//
// Wire format — cvc::volume (codec id "cvc.volume.v1"):
//   Header (little-endian):
//     magic[4]   = 'C','V','V','1'
//     xdim       = uint64
//     ydim       = uint64
//     zdim       = uint64
//     voxel_type = uint32 (data_type enum)
//     minx,miny,minz,maxx,maxy,maxz = double[6]
//     desc_len   = uint32
//     desc_bytes[desc_len]
//   Body:
//     raw voxel bytes (XDim*YDim*ZDim*voxelSize)
//
// Wire format — cvc::geometry (codec id "cvc.geometry.v1"):
//   Header (little-endian):
//     magic[4]     = 'C','V','G','1'
//     geom_type    = uint32 (geometry_type enum)
//     num_points   = uint64
//     num_normals  = uint64
//     num_colors   = uint64
//     num_tris     = uint64
//     num_quads    = uint64
//     num_tets     = uint64
//     num_hexs     = uint64
//     num_lines    = uint64
//   Body:
//     points[num_points]    — 3 x double each
//     normals[num_normals]  — 3 x double each
//     colors[num_colors]    — 3 x double each
//     tris[num_tris]        — 3 x uint64 each
//     quads[num_quads]      — 4 x uint64 each
//     tets[num_tets]        — 4 x uint64 each
//     hexs[num_hexs]        — 8 x uint64 each
//     lines[num_lines]      — 2 x uint64 each
//

// Register volume and geometry codecs on `registry`. Idempotent.
void register_volume_geometry_codecs(state_codec_registry &registry);

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_VOLUME_CODEC_H__
