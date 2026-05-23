/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_BRICK_MANIFEST_H__
#define __CVC_STATE_BRICK_MANIFEST_H__

#include <cstdint>
#include <cvc/namespace.h>
#include <cvc/state_blob_store.h>
#include <cvc/state_chunked_blob.h>
#include <cvc/state_compression_registry.h>
#include <string>
#include <vector>

namespace CVC_NAMESPACE {

// ----------------
// cvc::brick_extent
// ----------------
// Spatial metadata for a single chunk within a brick-aware manifest.
// Describes where in the logical volume (or geometry vertex range)
// this chunk's data lives.
//
struct brick_extent {
  // Volume bricks: origin voxel in the source volume.
  std::uint64_t origin_x = 0;
  std::uint64_t origin_y = 0;
  std::uint64_t origin_z = 0;

  // Volume bricks: size in voxels along each axis.
  std::uint64_t size_x = 0;
  std::uint64_t size_y = 0;
  std::uint64_t size_z = 0;

  // Variable index (for multi-variable datasets).
  std::uint32_t variable_index = 0;

  // Geometry chunks: vertex/element range [begin, end).
  std::uint64_t element_begin = 0;
  std::uint64_t element_end = 0;
};

// ----------------
// cvc::state_brick_manifest
// ----------------
// Extension of state_chunk_manifest that adds per-chunk spatial
// metadata. This enables receivers to fetch only the bricks they
// need (e.g. visible volume subregions, nearby geometry) rather
// than downloading the entire payload.
//
// Wire format (little-endian):
//   magic[4]    = 'C','V','B','1'
//   version     = uint32 (currently 1)
//   chunk_size  = uint32 (nominal chunk size in bytes)
//   total_size  = uint64 (sum of uncompressed chunk sizes)
//   codec_len   = uint32; codec[codec_len]
//   content_len = uint32; content_digest[content_len]
//
//   Volume header (present when has_volume_header == true):
//     has_volume_header = uint8 (1)
//     vol_xdim, vol_ydim, vol_zdim = uint64[3]
//     voxel_type   = uint32
//     brick_xdim, brick_ydim, brick_zdim = uint64[3]
//   else:
//     has_volume_header = uint8 (0)
//
//   count = uint32
//   for each chunk:
//     digest_len   = uint32; digest[digest_len]
//     stored_bytes = uint32
//     origin_x,y,z = uint64[3]
//     size_x,y,z   = uint64[3]
//     variable_index = uint32
//     element_begin  = uint64
//     element_end    = uint64
//
struct state_brick_manifest {
  std::uint32_t version = 1;
  std::uint32_t chunk_size = 0;
  std::uint64_t total_size = 0;
  std::string codec;
  std::string content_digest;

  // Per-chunk data.
  std::vector<std::string> chunks;
  std::vector<std::uint32_t> chunk_bytes;
  std::vector<brick_extent> extents;

  // Optional volume grid header.
  bool has_volume_header = false;
  std::uint64_t vol_xdim = 0, vol_ydim = 0, vol_zdim = 0;
  std::uint32_t voxel_type = 0;
  std::uint64_t brick_xdim = 0, brick_ydim = 0, brick_zdim = 0;

  std::vector<unsigned char> serialize() const;
  static bool parse(const std::vector<unsigned char> &bytes, state_brick_manifest &out);

  // Query: return indices of chunks whose spatial extent overlaps
  // the given axis-aligned box [lo, hi) in voxel coordinates.
  std::vector<std::size_t> bricks_in_region(std::uint64_t lo_x, std::uint64_t lo_y,
                                            std::uint64_t lo_z, std::uint64_t hi_x,
                                            std::uint64_t hi_y, std::uint64_t hi_z) const;

  // A half-space plane Ax+By+Cz+D >= 0.
  struct plane {
    double a, b, c, d;
  };

  // Query: return indices of chunks whose AABB is NOT completely
  // outside all 6 frustum planes (conservative — false positives
  // are possible). Each plane's positive half-space is "inside".
  std::vector<std::size_t> bricks_in_frustum(const plane planes[6]) const;
};

// ----------------
// cvc::state_brick_writer
// ----------------
// Splits a volume payload into spatially-addressed bricks and
// writes each brick as a content-addressed chunk in the blob store.
//
class state_brick_writer {
public:
  explicit state_brick_writer(state_blob_store &store, std::uint32_t brick_dim = 32,
                              const state_compression_registry *compression = nullptr);

  // Split the raw voxel `bytes` of a volume with the given grid
  // dimensions and voxel size into bricks and write them. Returns
  // the manifest blob_ref.
  state_blob_ref put_volume(const unsigned char *voxel_data, std::uint64_t xdim, std::uint64_t ydim,
                            std::uint64_t zdim, std::uint32_t voxel_size, std::uint32_t voxel_type,
                            const std::string &codec = std::string());

  // Write a geometry payload as element-range-addressed chunks.
  // `bytes` is the raw serialized geometry bytes, partitioned
  // into `bytes.size() / chunk_byte_size` chunks. Returns the
  // manifest blob_ref.
  state_blob_ref put_geometry(const std::vector<unsigned char> &bytes, std::uint64_t num_elements,
                              const std::string &codec = std::string());

  state_blob_store &store() { return _store; }
  std::uint32_t brick_dim() const { return _brick_dim; }

private:
  state_blob_store &_store;
  std::uint32_t _brick_dim;
  const state_compression_registry *_compression;
};

// ----------------
// cvc::state_brick_reader
// ----------------
// Reads a brick manifest and can reassemble the full payload or
// fetch individual bricks by spatial query.
//
class state_brick_reader {
public:
  explicit state_brick_reader(state_blob_store &store,
                              const state_compression_registry *compression = nullptr);

  // Load+parse a brick manifest by digest.
  bool load_manifest(const std::string &digest, state_brick_manifest &out) const;

  // Which chunks are missing from the local store.
  std::vector<std::string> missing_chunks(const state_brick_manifest &m) const;

  // Reassemble the full payload. Returns false if any chunk is
  // missing.
  bool get(const state_brick_manifest &m, std::vector<unsigned char> &out) const;

  // Fetch a single brick chunk by index. Returns false if the
  // chunk is not in the local store.
  bool get_brick(const state_brick_manifest &m, std::size_t brick_index,
                 std::vector<unsigned char> &out) const;

  // Fetch bricks that overlap a spatial region and write them into
  // a pre-allocated voxel buffer. Returns the number of bricks
  // fetched. Missing bricks are skipped (partial result).
  std::size_t get_region(const state_brick_manifest &m, unsigned char *dest_buffer,
                         std::uint64_t dest_xdim, std::uint64_t dest_ydim, std::uint64_t dest_zdim,
                         std::uint32_t voxel_size, std::uint64_t lo_x, std::uint64_t lo_y,
                         std::uint64_t lo_z, std::uint64_t hi_x, std::uint64_t hi_y,
                         std::uint64_t hi_z) const;

  state_blob_store &store() { return _store; }

private:
  state_blob_store &_store;
  const state_compression_registry *_compression;
};

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_BRICK_MANIFEST_H__
