/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cstring>
#include <cvc/state_brick_manifest.h>
#include <cvc/types.h>
#include <stdexcept>

namespace CVC_NAMESPACE {

namespace {

constexpr unsigned char BRICK_MAGIC[4] = {'C', 'V', 'B', '1'};

void wr_u8(std::vector<unsigned char> &o, std::uint8_t v) { o.push_back(v); }
void wr_u32(std::vector<unsigned char> &o, std::uint32_t v) {
  for (int i = 0; i < 4; ++i)
    o.push_back(static_cast<unsigned char>((v >> (8 * i)) & 0xff));
}
void wr_u64(std::vector<unsigned char> &o, std::uint64_t v) {
  for (int i = 0; i < 8; ++i)
    o.push_back(static_cast<unsigned char>((v >> (8 * i)) & 0xff));
}
void wr_str(std::vector<unsigned char> &o, const std::string &s) {
  wr_u32(o, static_cast<std::uint32_t>(s.size()));
  o.insert(o.end(), s.begin(), s.end());
}

bool rd_u8(const unsigned char *&p, const unsigned char *e, std::uint8_t &v) {
  if (p >= e)
    return false;
  v = *p++;
  return true;
}
bool rd_u32(const unsigned char *&p, const unsigned char *e, std::uint32_t &v) {
  if (p + 4 > e)
    return false;
  v = 0;
  for (int i = 0; i < 4; ++i)
    v |= static_cast<std::uint32_t>(p[i]) << (8 * i);
  p += 4;
  return true;
}
bool rd_u64(const unsigned char *&p, const unsigned char *e, std::uint64_t &v) {
  if (p + 8 > e)
    return false;
  v = 0;
  for (int i = 0; i < 8; ++i)
    v |= static_cast<std::uint64_t>(p[i]) << (8 * i);
  p += 8;
  return true;
}
bool rd_str(const unsigned char *&p, const unsigned char *e, std::string &v) {
  std::uint32_t n;
  if (!rd_u32(p, e, n))
    return false;
  if (p + n > e)
    return false;
  v.assign(reinterpret_cast<const char *>(p), n);
  p += n;
  return true;
}

} // namespace

// ---------- state_brick_manifest ----------

std::vector<unsigned char> state_brick_manifest::serialize() const {
  std::vector<unsigned char> out;
  out.reserve(128 + chunks.size() * 128);

  out.insert(out.end(), BRICK_MAGIC, BRICK_MAGIC + 4);
  wr_u32(out, version);
  wr_u32(out, chunk_size);
  wr_u64(out, total_size);
  wr_str(out, codec);
  wr_str(out, content_digest);

  wr_u8(out, has_volume_header ? 1 : 0);
  if (has_volume_header) {
    wr_u64(out, vol_xdim);
    wr_u64(out, vol_ydim);
    wr_u64(out, vol_zdim);
    wr_u32(out, voxel_type);
    wr_u64(out, brick_xdim);
    wr_u64(out, brick_ydim);
    wr_u64(out, brick_zdim);
  }

  wr_u32(out, static_cast<std::uint32_t>(chunks.size()));
  for (std::size_t i = 0; i < chunks.size(); ++i) {
    wr_str(out, chunks[i]);
    wr_u32(out, i < chunk_bytes.size() ? chunk_bytes[i] : 0u);

    const brick_extent &ext = i < extents.size() ? extents[i] : brick_extent{};
    wr_u64(out, ext.origin_x);
    wr_u64(out, ext.origin_y);
    wr_u64(out, ext.origin_z);
    wr_u64(out, ext.size_x);
    wr_u64(out, ext.size_y);
    wr_u64(out, ext.size_z);
    wr_u32(out, ext.variable_index);
    wr_u64(out, ext.element_begin);
    wr_u64(out, ext.element_end);
  }
  return out;
}

bool state_brick_manifest::parse(const std::vector<unsigned char> &bytes,
                                 state_brick_manifest &out) {
  if (bytes.size() < 4)
    return false;
  const unsigned char *p = bytes.data();
  const unsigned char *e = p + bytes.size();

  if (std::memcmp(p, BRICK_MAGIC, 4) != 0)
    return false;
  p += 4;

  if (!rd_u32(p, e, out.version) || out.version != 1)
    return false;
  if (!rd_u32(p, e, out.chunk_size))
    return false;
  if (!rd_u64(p, e, out.total_size))
    return false;
  if (!rd_str(p, e, out.codec))
    return false;
  if (!rd_str(p, e, out.content_digest))
    return false;

  std::uint8_t has_vh;
  if (!rd_u8(p, e, has_vh))
    return false;
  out.has_volume_header = (has_vh != 0);
  if (out.has_volume_header) {
    if (!rd_u64(p, e, out.vol_xdim) || !rd_u64(p, e, out.vol_ydim) || !rd_u64(p, e, out.vol_zdim))
      return false;
    if (!rd_u32(p, e, out.voxel_type))
      return false;
    if (!rd_u64(p, e, out.brick_xdim) || !rd_u64(p, e, out.brick_ydim) ||
        !rd_u64(p, e, out.brick_zdim))
      return false;
  }

  std::uint32_t count;
  if (!rd_u32(p, e, count))
    return false;

  out.chunks.clear();
  out.chunks.reserve(count);
  out.chunk_bytes.clear();
  out.chunk_bytes.reserve(count);
  out.extents.clear();
  out.extents.reserve(count);

  for (std::uint32_t i = 0; i < count; ++i) {
    std::string digest;
    std::uint32_t sb;
    if (!rd_str(p, e, digest))
      return false;
    if (!rd_u32(p, e, sb))
      return false;

    brick_extent ext;
    if (!rd_u64(p, e, ext.origin_x) || !rd_u64(p, e, ext.origin_y) || !rd_u64(p, e, ext.origin_z))
      return false;
    if (!rd_u64(p, e, ext.size_x) || !rd_u64(p, e, ext.size_y) || !rd_u64(p, e, ext.size_z))
      return false;
    if (!rd_u32(p, e, ext.variable_index))
      return false;
    if (!rd_u64(p, e, ext.element_begin) || !rd_u64(p, e, ext.element_end))
      return false;

    out.chunks.push_back(std::move(digest));
    out.chunk_bytes.push_back(sb);
    out.extents.push_back(ext);
  }
  return true;
}

std::vector<std::size_t>
state_brick_manifest::bricks_in_region(std::uint64_t lo_x, std::uint64_t lo_y, std::uint64_t lo_z,
                                       std::uint64_t hi_x, std::uint64_t hi_y,
                                       std::uint64_t hi_z) const {
  std::vector<std::size_t> result;
  for (std::size_t i = 0; i < extents.size(); ++i) {
    const brick_extent &ext = extents[i];
    // AABB overlap test.
    if (ext.origin_x + ext.size_x <= lo_x || ext.origin_x >= hi_x)
      continue;
    if (ext.origin_y + ext.size_y <= lo_y || ext.origin_y >= hi_y)
      continue;
    if (ext.origin_z + ext.size_z <= lo_z || ext.origin_z >= hi_z)
      continue;
    result.push_back(i);
  }
  return result;
}

std::vector<std::size_t> state_brick_manifest::bricks_in_frustum(const plane planes[6]) const {
  std::vector<std::size_t> result;
  for (std::size_t i = 0; i < extents.size(); ++i) {
    const brick_extent &ext = extents[i];
    double lo_x = static_cast<double>(ext.origin_x);
    double lo_y = static_cast<double>(ext.origin_y);
    double lo_z = static_cast<double>(ext.origin_z);
    double hi_x = lo_x + static_cast<double>(ext.size_x);
    double hi_y = lo_y + static_cast<double>(ext.size_y);
    double hi_z = lo_z + static_cast<double>(ext.size_z);

    bool outside = false;
    for (int p = 0; p < 6 && !outside; ++p) {
      // Find the AABB corner most aligned with the plane normal
      // (the "positive vertex"). If that corner is outside the
      // plane, the entire AABB is outside this half-space.
      double px = (planes[p].a >= 0) ? hi_x : lo_x;
      double py = (planes[p].b >= 0) ? hi_y : lo_y;
      double pz = (planes[p].c >= 0) ? hi_z : lo_z;
      if (planes[p].a * px + planes[p].b * py + planes[p].c * pz + planes[p].d < 0)
        outside = true;
    }
    if (!outside)
      result.push_back(i);
  }
  return result;
}

// ---------- state_brick_writer ----------

state_brick_writer::state_brick_writer(state_blob_store &store, std::uint32_t brick_dim,
                                       const state_compression_registry *compression)
    : _store(store), _brick_dim(brick_dim == 0 ? 1u : brick_dim), _compression(compression) {}

state_blob_ref state_brick_writer::put_volume(const unsigned char *voxel_data, std::uint64_t xdim,
                                              std::uint64_t ydim, std::uint64_t zdim,
                                              std::uint32_t voxel_size, std::uint32_t voxel_type,
                                              const std::string &codec) {
  state_brick_manifest m;
  m.version = 1;
  m.chunk_size = _brick_dim;
  m.total_size = xdim * ydim * zdim * voxel_size;
  m.codec = codec;

  // Compute content_digest over the full voxel data.
  m.content_digest = sha256_hex(voxel_data, static_cast<std::size_t>(m.total_size));

  m.has_volume_header = true;
  m.vol_xdim = xdim;
  m.vol_ydim = ydim;
  m.vol_zdim = zdim;
  m.voxel_type = voxel_type;
  m.brick_xdim = _brick_dim;
  m.brick_ydim = _brick_dim;
  m.brick_zdim = _brick_dim;

  // Resolve compression codec.
  std::shared_ptr<state_compression_codec> compressor;
  if (!codec.empty() && codec != "raw" && _compression) {
    compressor = _compression->get(codec);
    if (!compressor)
      throw std::runtime_error("state_brick_writer: unknown compression codec: " + codec);
  }

  // Iterate over the 3D brick grid.
  for (std::uint64_t bz = 0; bz < zdim; bz += _brick_dim) {
    for (std::uint64_t by = 0; by < ydim; by += _brick_dim) {
      for (std::uint64_t bx = 0; bx < xdim; bx += _brick_dim) {
        const std::uint64_t sx = std::min<std::uint64_t>(_brick_dim, xdim - bx);
        const std::uint64_t sy = std::min<std::uint64_t>(_brick_dim, ydim - by);
        const std::uint64_t sz = std::min<std::uint64_t>(_brick_dim, zdim - bz);

        // Extract the brick from the linear volume.
        std::vector<unsigned char> brick(static_cast<std::size_t>(sx * sy * sz * voxel_size));
        for (std::uint64_t z = 0; z < sz; ++z) {
          for (std::uint64_t y = 0; y < sy; ++y) {
            const std::uint64_t src_off =
                ((bz + z) * ydim * xdim + (by + y) * xdim + bx) * voxel_size;
            const std::uint64_t dst_off = (z * sy * sx + y * sx) * voxel_size;
            std::memcpy(brick.data() + dst_off, voxel_data + src_off,
                        static_cast<std::size_t>(sx * voxel_size));
          }
        }

        // Compress.
        std::vector<unsigned char> stored;
        if (compressor) {
          stored = compressor->encode(brick);
        } else {
          stored = std::move(brick);
        }

        state_blob_ref ref = _store.put(stored);
        m.chunks.push_back(ref.digest);
        m.chunk_bytes.push_back(static_cast<std::uint32_t>(stored.size()));

        brick_extent ext;
        ext.origin_x = bx;
        ext.origin_y = by;
        ext.origin_z = bz;
        ext.size_x = sx;
        ext.size_y = sy;
        ext.size_z = sz;
        m.extents.push_back(ext);
      }
    }
  }

  auto manifest_bytes = m.serialize();
  return _store.put(manifest_bytes, codec);
}

state_blob_ref state_brick_writer::put_geometry(const std::vector<unsigned char> &bytes,
                                                std::uint64_t num_elements,
                                                const std::string &codec) {
  state_brick_manifest m;
  m.version = 1;
  m.total_size = bytes.size();
  m.codec = codec;
  m.content_digest = sha256_hex(bytes);
  m.has_volume_header = false;

  std::shared_ptr<state_compression_codec> compressor;
  if (!codec.empty() && codec != "raw" && _compression) {
    compressor = _compression->get(codec);
    if (!compressor)
      throw std::runtime_error("state_brick_writer: unknown compression codec: " + codec);
  }

  // Partition into fixed-size byte chunks; assign element ranges
  // proportionally.
  const std::uint32_t chunk_sz = _brick_dim * _brick_dim * _brick_dim; // reuse brick_dim^3
  m.chunk_size = chunk_sz;
  std::size_t off = 0;
  std::uint64_t elem_cursor = 0;

  while (off < bytes.size()) {
    const std::size_t n = std::min<std::size_t>(chunk_sz, bytes.size() - off);
    std::vector<unsigned char> chunk(bytes.begin() + off, bytes.begin() + off + n);

    std::vector<unsigned char> stored;
    if (compressor) {
      stored = compressor->encode(chunk);
    } else {
      stored = std::move(chunk);
    }

    state_blob_ref ref = _store.put(stored);
    m.chunks.push_back(ref.digest);
    m.chunk_bytes.push_back(static_cast<std::uint32_t>(stored.size()));

    // Estimate element range proportionally.
    std::uint64_t elem_end =
        (bytes.size() == 0) ? 0 : std::min(num_elements, (off + n) * num_elements / bytes.size());

    brick_extent ext;
    ext.element_begin = elem_cursor;
    ext.element_end = elem_end;
    m.extents.push_back(ext);

    elem_cursor = elem_end;
    off += n;
  }

  auto manifest_bytes = m.serialize();
  return _store.put(manifest_bytes, codec);
}

// ---------- state_brick_reader ----------

state_brick_reader::state_brick_reader(state_blob_store &store,
                                       const state_compression_registry *compression)
    : _store(store), _compression(compression) {}

bool state_brick_reader::load_manifest(const std::string &digest, state_brick_manifest &out) const {
  std::vector<unsigned char> bytes;
  if (!_store.get(digest, bytes))
    return false;
  return state_brick_manifest::parse(bytes, out);
}

std::vector<std::string> state_brick_reader::missing_chunks(const state_brick_manifest &m) const {
  std::vector<std::string> missing;
  for (const auto &d : m.chunks) {
    if (!_store.has(d))
      missing.push_back(d);
  }
  return missing;
}

bool state_brick_reader::get(const state_brick_manifest &m, std::vector<unsigned char> &out) const {
  if (!m.has_volume_header) {
    // Non-volume: concatenate chunks in order.
    out.clear();
    out.reserve(static_cast<std::size_t>(m.total_size));

    std::shared_ptr<state_compression_codec> decompressor;
    if (!m.codec.empty() && m.codec != "raw" && _compression) {
      decompressor = _compression->get(m.codec);
      if (!decompressor)
        return false;
    }

    for (const auto &d : m.chunks) {
      std::vector<unsigned char> stored;
      if (!_store.get(d, stored))
        return false;
      if (decompressor) {
        std::vector<unsigned char> decoded;
        if (!decompressor->decode(stored, decoded))
          return false;
        out.insert(out.end(), decoded.begin(), decoded.end());
      } else {
        out.insert(out.end(), stored.begin(), stored.end());
      }
    }
    return true;
  }

  // Volume: scatter bricks into a linear voxel buffer.
  const std::uint64_t vs = data_type_sizes[m.voxel_type];
  out.assign(static_cast<std::size_t>(m.vol_xdim * m.vol_ydim * m.vol_zdim * vs), 0);

  std::shared_ptr<state_compression_codec> decompressor;
  if (!m.codec.empty() && m.codec != "raw" && _compression) {
    decompressor = _compression->get(m.codec);
    if (!decompressor)
      return false;
  }

  for (std::size_t i = 0; i < m.chunks.size(); ++i) {
    std::vector<unsigned char> stored;
    if (!_store.get(m.chunks[i], stored))
      return false;

    std::vector<unsigned char> brick;
    if (decompressor) {
      if (!decompressor->decode(stored, brick))
        return false;
    } else {
      brick = std::move(stored);
    }

    const brick_extent &ext = m.extents[i];
    for (std::uint64_t z = 0; z < ext.size_z; ++z) {
      for (std::uint64_t y = 0; y < ext.size_y; ++y) {
        const std::uint64_t dst_off = ((ext.origin_z + z) * m.vol_ydim * m.vol_xdim +
                                       (ext.origin_y + y) * m.vol_xdim + ext.origin_x) *
                                      vs;
        const std::uint64_t src_off = (z * ext.size_y * ext.size_x + y * ext.size_x) * vs;
        std::memcpy(out.data() + dst_off, brick.data() + src_off,
                    static_cast<std::size_t>(ext.size_x * vs));
      }
    }
  }

  if (!m.content_digest.empty() && sha256_hex(out) != m.content_digest) {
    out.clear();
    return false;
  }
  return true;
}

bool state_brick_reader::get_brick(const state_brick_manifest &m, std::size_t brick_index,
                                   std::vector<unsigned char> &out) const {
  if (brick_index >= m.chunks.size())
    return false;

  std::vector<unsigned char> stored;
  if (!_store.get(m.chunks[brick_index], stored))
    return false;

  if (!m.codec.empty() && m.codec != "raw" && _compression) {
    auto dec = _compression->get(m.codec);
    if (!dec)
      return false;
    return dec->decode(stored, out);
  }
  out = std::move(stored);
  return true;
}

std::size_t state_brick_reader::get_region(const state_brick_manifest &m,
                                           unsigned char *dest_buffer, std::uint64_t dest_xdim,
                                           std::uint64_t dest_ydim, std::uint64_t dest_zdim,
                                           std::uint32_t voxel_size, std::uint64_t lo_x,
                                           std::uint64_t lo_y, std::uint64_t lo_z,
                                           std::uint64_t hi_x, std::uint64_t hi_y,
                                           std::uint64_t hi_z) const {
  auto indices = m.bricks_in_region(lo_x, lo_y, lo_z, hi_x, hi_y, hi_z);
  std::size_t fetched = 0;

  for (std::size_t idx : indices) {
    std::vector<unsigned char> brick;
    if (!get_brick(m, idx, brick))
      continue; // skip missing bricks

    const brick_extent &ext = m.extents[idx];

    // Compute overlap region.
    const std::uint64_t ox = std::max(ext.origin_x, lo_x);
    const std::uint64_t oy = std::max(ext.origin_y, lo_y);
    const std::uint64_t oz = std::max(ext.origin_z, lo_z);
    const std::uint64_t ex = std::min(ext.origin_x + ext.size_x, hi_x);
    const std::uint64_t ey = std::min(ext.origin_y + ext.size_y, hi_y);
    const std::uint64_t ez = std::min(ext.origin_z + ext.size_z, hi_z);

    for (std::uint64_t z = oz; z < ez; ++z) {
      for (std::uint64_t y = oy; y < ey; ++y) {
        const std::uint64_t src_off = ((z - ext.origin_z) * ext.size_y * ext.size_x +
                                       (y - ext.origin_y) * ext.size_x + (ox - ext.origin_x)) *
                                      voxel_size;
        const std::uint64_t dst_off =
            ((z - lo_z) * dest_ydim * dest_xdim + (y - lo_y) * dest_xdim + (ox - lo_x)) *
            voxel_size;
        const std::uint64_t copy_len = (ex - ox) * voxel_size;
        if (src_off + copy_len <= brick.size() &&
            dst_off + copy_len <= dest_xdim * dest_ydim * dest_zdim * voxel_size) {
          std::memcpy(dest_buffer + dst_off, brick.data() + src_off,
                      static_cast<std::size_t>(copy_len));
        }
      }
    }
    ++fetched;
  }
  return fetched;
}

} // namespace CVC_NAMESPACE
