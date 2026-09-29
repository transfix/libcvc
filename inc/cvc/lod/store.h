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

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// store.h -- persist LOD pyramids into a per-bundle "scene.cvch5" HDF5 file, and
// read them back. This is the cache/serialization half named alongside the
// pyramid_builder (volrover3 section 22.1); it fills the `/cvc/geometry`
// placeholder the CVC HDF5 schema reserved (hdf5_utils.h, "define fully later").
//
// Layout (one file per bundle):
//   /cvc/geometry/<name>          group; attrs: kind='M', nrungs, source_hash,
//                                 world_error_m (nrungs doubles = the ladder)
//   /cvc/geometry/<name>/lod/<k>  group; datasets: points (Nx3 f64), tris (Mx3 u64),
//                                 uv (Nx2 f64, optional), colors (Nx3 f64, optional);
//                                 attrs: world_error_m, ntris
//   /cvc/images/<name>            group; attrs: kind='I', nrungs, source_hash, world_error_m
//   /cvc/images/<name>/lod/<k>    dataset pixels (H x W x C u8); attrs: w, h, channels,
//                                 format, world_error_m
//
// The HDF5 hierarchy IS the lod_index: read_lod_index() walks these groups. A
// content hash on the SOURCE asset lets a bake step skip a pyramid that is already
// present and current (has_pyramid). Requires CVC_USING_HDF5; the whole header is
// a no-op declaration otherwise (the .cpp is compiled only with HDF5).
//
// LOD is a render proxy: a scene.cvch5 never feeds a nav/material/RF path.

#ifndef __CVC_LOD_STORE_H__
#define __CVC_LOD_STORE_H__

#include <cvc/lod/pyramid.h>
#include <string>
#include <vector>

namespace cvc {
class app;

namespace lod {

// One asset's presence in a scene.cvch5, derived by walking the file's groups.
struct lod_index_entry {
  std::string name;
  char kind = 'M'; // 'M' mesh, 'I' image, 'V' volume
  int nrungs = 0;  // including rung 0
  std::vector<double> world_error_m;
  std::string source_hash;
};

// Content hashes of a source asset's defining bytes -- the detect-vs-rebuild key.
std::string mesh_content_hash(const geometry &g);
std::string image_content_hash(const image &img);

// Write / read a mesh pyramid under /cvc/geometry/<name>. write creates the file
// if needed and replaces any existing pyramid at that name.
void write_mesh_pyramid(app &ctx, const std::string &h5file, const std::string &name,
                        const mesh_pyramid &pyr, const std::string &source_hash = std::string());
mesh_pyramid read_mesh_pyramid(app &ctx, const std::string &h5file, const std::string &name);

// Write / read an image mip pyramid under /cvc/images/<name>.
void write_image_pyramid(app &ctx, const std::string &h5file, const std::string &name,
                         const image_pyramid &pyr, const std::string &source_hash = std::string());
image_pyramid read_image_pyramid(app &ctx, const std::string &h5file, const std::string &name);

// Walk the file and return every asset's index entry (any kind).
std::vector<lod_index_entry> read_lod_index(app &ctx, const std::string &h5file);

// True when <name> is present with a matching source_hash (skip the rebuild).
bool has_pyramid(app &ctx, const std::string &h5file, const std::string &name,
                 const std::string &source_hash);

} // namespace lod
} // namespace cvc

#endif // __CVC_LOD_STORE_H__
