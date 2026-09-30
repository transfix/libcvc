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

// store.h -- persist LOD pyramids into a "scene.cvch5" HDF5 container, on disk OR
// entirely in memory (no file). This is the cache/serialization half named
// alongside the pyramid_builder (volrover3 section 22.1); it fills the
// `/cvc/geometry` group the CVC HDF5 schema reserved (hdf5_utils.h).
//
// Layout (identical for a file or an in-memory image):
//   /cvc/geometry/<name>          group; attrs: kind='M', nrungs, source_hash,
//                                 world_error_m (nrungs doubles = the ladder)
//   /cvc/geometry/<name>/lod/<k>  group; datasets: points (Nx3 f64), tris (Mx3 u64),
//                                 uv (Nx2 f64, optional), colors (Nx3 f64, optional);
//                                 attrs: world_error_m, ntris
//   /cvc/images/<name>            group; attrs: kind='I', nrungs, source_hash, world_error_m
//   /cvc/images/<name>/lod/<k>    dataset pixels (H x W x C u8); attrs: w, h, channels,
//                                 format, world_error_m
//
// Two ways in:
//   * FILE      -- the free write_/read_*_pyramid functions and scene_writer/
//                  scene_reader(ctx, path). Standard HDF5 file IO.
//   * IN-MEMORY -- scene_writer(ctx) builds the whole container in RAM (HDF5's core
//                  VFD) and to_blob() hands back the exact bytes an on-disk file
//                  would have, ready to POST over the network; scene_reader(ctx,
//                  bytes, len) opens such a blob (e.g. one fetched with cvc::net)
//                  with no temp file. A blob is byte-identical to the file image,
//                  so either side can produce what the other consumes.
//
// The HDF5 hierarchy IS the lod_index (read_lod_index / index() walk it); a
// content hash on the SOURCE asset lets a bake step skip a pyramid already present
// and current (has_pyramid / has). Requires CVC_USING_HDF5.
//
// Any number of writers and readers may be alive at once and used from any
// thread (but see scene_reader on sharing a file with a writer); each in-memory
// container is its own. Every call goes through the process-wide
// hdf5_utils::library_lock (destruction included), so a writer/reader must not
// outlive its app. Failures -- a missing, corrupt or truncated file or blob, an
// absent asset -- throw cvc::hdf5_exception, a std::exception; HDF5's own
// H5::Exception never escapes.
//
// LOD is a render proxy: a scene.cvch5 never feeds a nav/material/RF path.

#ifndef __CVC_LOD_STORE_H__
#define __CVC_LOD_STORE_H__

#include <cstddef>
#include <cvc/lod/pyramid.h>
#include <memory>
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

// ── writer ────────────────────────────────────────────────────────────────
// Accumulates pyramids into one scene.cvch5. File-backed: writes land on disk as
// each call returns (an existing scene is opened, preserving prior assets; a
// missing one is created). It never truncates: an existing path it cannot open
// read-write -- not HDF5, damaged, read-only, or open in a live scene_reader --
// throws rather than being replaced. In-memory: nothing touches the disk until
// to_blob() serializes the container to bytes.
class scene_writer {
public:
  explicit scene_writer(app &ctx);                 // in-memory (core VFD, no file)
  scene_writer(app &ctx, const std::string &path); // on disk at `path`
  ~scene_writer();
  scene_writer(scene_writer &&) noexcept;
  scene_writer &operator=(scene_writer &&) noexcept;

  void write_mesh_pyramid(const std::string &name, const mesh_pyramid &pyr,
                          const std::string &source_hash = std::string());
  void write_image_pyramid(const std::string &name, const image_pyramid &pyr,
                           const std::string &source_hash = std::string());

  // The container's bytes -- exactly what an on-disk scene.cvch5 holds. Valid for
  // both in-memory and file-backed writers (a file writer flushes then images
  // itself), so a bake can build once and both save and POST it.
  std::vector<unsigned char> to_blob();

private:
  struct impl;
  std::unique_ptr<impl> _p;
};

// ── reader ────────────────────────────────────────────────────────────────
// Reads pyramids from a scene.cvch5 -- a file on disk, or a blob in RAM (e.g. a
// body fetched with cvc::net), with no temp file for the blob case. A file is
// opened read-only and must already exist: a reader never creates or modifies
// it (so a file writer cannot open that path while the reader is alive). A blob
// is copied, so its bytes need not outlive the constructor.
class scene_reader {
public:
  scene_reader(app &ctx, const std::string &path);                   // from a file
  scene_reader(app &ctx, const unsigned char *bytes, std::size_t n); // from a blob
  ~scene_reader();
  scene_reader(scene_reader &&) noexcept;
  scene_reader &operator=(scene_reader &&) noexcept;

  mesh_pyramid read_mesh_pyramid(const std::string &name);
  image_pyramid read_image_pyramid(const std::string &name);
  std::vector<lod_index_entry> index();
  bool has(const std::string &name, const std::string &source_hash = std::string());

private:
  struct impl;
  std::unique_ptr<impl> _p;
};

// ── free functions (file only; thin wrappers over the writer/reader) ────────
void write_mesh_pyramid(app &ctx, const std::string &h5file, const std::string &name,
                        const mesh_pyramid &pyr, const std::string &source_hash = std::string());
mesh_pyramid read_mesh_pyramid(app &ctx, const std::string &h5file, const std::string &name);
void write_image_pyramid(app &ctx, const std::string &h5file, const std::string &name,
                         const image_pyramid &pyr, const std::string &source_hash = std::string());
image_pyramid read_image_pyramid(app &ctx, const std::string &h5file, const std::string &name);
std::vector<lod_index_entry> read_lod_index(app &ctx, const std::string &h5file);
bool has_pyramid(app &ctx, const std::string &h5file, const std::string &name,
                 const std::string &source_hash);

// ── bake ────────────────────────────────────────────────────────────────────
// Build a mesh's LOD pyramid and write it into `h5file` under `name`, SKIPPING the
// work when a current pyramid (one whose source_hash matches this mesh) is already
// present -- unless `force`. `pool` fans the pyramid build over the app workers.
// Returns true if it (re)baked, false if it skipped an up-to-date pyramid. This is
// the reusable core of the cvc-lod-bake tool.
bool bake_mesh_asset(app &ctx, const std::string &h5file, const std::string &name,
                     const geometry &src, const pyramid_params &params = pyramid_params(),
                     bool force = false, thread_pool *pool = nullptr);

} // namespace lod
} // namespace cvc

#endif // __CVC_LOD_STORE_H__
