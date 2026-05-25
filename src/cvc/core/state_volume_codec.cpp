/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <boost/any.hpp>
#include <cstring>
#include <cvc/app.h>
#include <cvc/geometry.h>
#include <cvc/state_volume_codec.h>
#include <cvc/volume.h>
#include <stdexcept>

namespace cvc {

namespace {

// ---- little-endian helpers ----

void write_u32(std::vector<unsigned char> &out, std::uint32_t v) {
  for (int i = 0; i < 4; ++i)
    out.push_back(static_cast<unsigned char>((v >> (8 * i)) & 0xff));
}

void write_u64(std::vector<unsigned char> &out, std::uint64_t v) {
  for (int i = 0; i < 8; ++i)
    out.push_back(static_cast<unsigned char>((v >> (8 * i)) & 0xff));
}

void write_f64(std::vector<unsigned char> &out, double v) {
  std::uint64_t bits;
  static_assert(sizeof(double) == 8, "double must be 8 bytes");
  std::memcpy(&bits, &v, 8);
  write_u64(out, bits);
}

void write_str(std::vector<unsigned char> &out, const std::string &s) {
  write_u32(out, static_cast<std::uint32_t>(s.size()));
  out.insert(out.end(), s.begin(), s.end());
}

void write_raw(std::vector<unsigned char> &out, const unsigned char *data, std::size_t len) {
  out.insert(out.end(), data, data + len);
}

bool read_u32(const unsigned char *&p, const unsigned char *end, std::uint32_t &v) {
  if (p + 4 > end)
    return false;
  v = 0;
  for (int i = 0; i < 4; ++i)
    v |= static_cast<std::uint32_t>(p[i]) << (8 * i);
  p += 4;
  return true;
}

bool read_u64(const unsigned char *&p, const unsigned char *end, std::uint64_t &v) {
  if (p + 8 > end)
    return false;
  v = 0;
  for (int i = 0; i < 8; ++i)
    v |= static_cast<std::uint64_t>(p[i]) << (8 * i);
  p += 8;
  return true;
}

bool read_f64(const unsigned char *&p, const unsigned char *end, double &v) {
  std::uint64_t bits;
  if (!read_u64(p, end, bits))
    return false;
  std::memcpy(&v, &bits, 8);
  return true;
}

bool read_str(const unsigned char *&p, const unsigned char *end, std::string &v) {
  std::uint32_t len;
  if (!read_u32(p, end, len))
    return false;
  if (p + len > end)
    return false;
  v.assign(reinterpret_cast<const char *>(p), len);
  p += len;
  return true;
}

// ---- volume codec ----

constexpr unsigned char VOL_MAGIC[4] = {'C', 'V', 'V', '1'};

std::vector<unsigned char> encode_volume(const boost::any &val) {
  const volume &vol = boost::any_cast<const volume &>(val);

  const std::uint64_t xdim = vol.XDim();
  const std::uint64_t ydim = vol.YDim();
  const std::uint64_t zdim = vol.ZDim();
  const std::uint32_t vtype = static_cast<std::uint32_t>(vol.voxelType());
  const std::uint64_t voxel_bytes = xdim * ydim * zdim * data_type_sizes[vol.voxelType()];

  std::vector<unsigned char> out;
  out.reserve(4 + 3 * 8 + 4 + 6 * 8 + 4 + vol.desc().size() + voxel_bytes);

  out.insert(out.end(), VOL_MAGIC, VOL_MAGIC + 4);
  write_u64(out, xdim);
  write_u64(out, ydim);
  write_u64(out, zdim);
  write_u32(out, vtype);

  write_f64(out, vol.XMin());
  write_f64(out, vol.YMin());
  write_f64(out, vol.ZMin());
  write_f64(out, vol.XMax());
  write_f64(out, vol.YMax());
  write_f64(out, vol.ZMax());

  write_str(out, vol.desc());

  const unsigned char *voxels = *vol;
  write_raw(out, voxels, static_cast<std::size_t>(voxel_bytes));

  return out;
}

boost::any decode_volume(const std::vector<unsigned char> &bytes) {
  const unsigned char *p = bytes.data();
  const unsigned char *end = p + bytes.size();

  if (bytes.size() < 4 || std::memcmp(p, VOL_MAGIC, 4) != 0)
    throw std::runtime_error("decode_volume: bad magic");
  p += 4;

  std::uint64_t xdim, ydim, zdim;
  std::uint32_t vtype;
  if (!read_u64(p, end, xdim) || !read_u64(p, end, ydim) || !read_u64(p, end, zdim) ||
      !read_u32(p, end, vtype))
    throw std::runtime_error("decode_volume: truncated header");

  if (vtype > static_cast<std::uint32_t>(Undefined))
    throw std::runtime_error("decode_volume: invalid voxel type");

  double minx, miny, minz, maxx, maxy, maxz;
  if (!read_f64(p, end, minx) || !read_f64(p, end, miny) || !read_f64(p, end, minz) ||
      !read_f64(p, end, maxx) || !read_f64(p, end, maxy) || !read_f64(p, end, maxz))
    throw std::runtime_error("decode_volume: truncated bounding box");

  std::string desc;
  if (!read_str(p, end, desc))
    throw std::runtime_error("decode_volume: truncated description");

  data_type dt = static_cast<data_type>(vtype);
  std::uint64_t voxel_bytes = xdim * ydim * zdim * data_type_sizes[dt];
  if (static_cast<std::size_t>(end - p) < voxel_bytes)
    throw std::runtime_error("decode_volume: truncated voxel data");

  // Construct volume via a scratch app context.
  // The caller is expected to extract the volume from the any and
  // manage ownership. We use a thread-local static app so we don't
  // create one per decode call.
  thread_local app decode_ctx;

  dimension dim;
  dim.xdim = xdim;
  dim.ydim = ydim;
  dim.zdim = zdim;
  bounding_box bbox(minx, miny, minz, maxx, maxy, maxz);

  volume vol(decode_ctx, p, dim, dt, bbox);
  vol.desc(desc);
  p += voxel_bytes;

  return boost::any(vol);
}

// ---- geometry codec ----

constexpr unsigned char GEO_MAGIC[4] = {'C', 'V', 'G', '1'};

std::vector<unsigned char> encode_geometry(const boost::any &val) {
  const geometry &geo = boost::any_cast<const geometry &>(val);

  const auto &pts = geo.const_points();
  const auto &nrm = geo.const_normals();
  const auto &clr = geo.const_colors();
  const auto &tri = geo.const_tris();
  const auto &qua = geo.const_quads();
  const auto &tet = geo.const_tets();
  const auto &hex = geo.const_hexs();
  const auto &lin = geo.const_lines();

  const std::uint64_t np = pts.size();
  const std::uint64_t nn = nrm.size();
  const std::uint64_t nc = clr.size();
  const std::uint64_t nt = tri.size();
  const std::uint64_t nq = qua.size();
  const std::uint64_t nte = tet.size();
  const std::uint64_t nh = hex.size();
  const std::uint64_t nl = lin.size();

  std::vector<unsigned char> out;
  // Header: 4 + 4 + 8*8 = 72 bytes
  // Body est: pts*24 + nrm*24 + clr*24 + tri*24 + qua*32 + tet*32 + hex*64 + lin*16
  out.reserve(72 + np * 24 + nn * 24 + nc * 24 + nt * 24 + nq * 32 + nte * 32 + nh * 64 + nl * 16);

  out.insert(out.end(), GEO_MAGIC, GEO_MAGIC + 4);
  write_u32(out, static_cast<std::uint32_t>(geo.get_geometry_type()));
  write_u64(out, np);
  write_u64(out, nn);
  write_u64(out, nc);
  write_u64(out, nt);
  write_u64(out, nq);
  write_u64(out, nte);
  write_u64(out, nh);
  write_u64(out, nl);

  for (const auto &pt : pts) {
    write_f64(out, pt[0]);
    write_f64(out, pt[1]);
    write_f64(out, pt[2]);
  }
  for (const auto &n : nrm) {
    write_f64(out, n[0]);
    write_f64(out, n[1]);
    write_f64(out, n[2]);
  }
  for (const auto &c : clr) {
    write_f64(out, c[0]);
    write_f64(out, c[1]);
    write_f64(out, c[2]);
  }
  for (const auto &t : tri) {
    write_u64(out, t[0]);
    write_u64(out, t[1]);
    write_u64(out, t[2]);
  }
  for (const auto &q : qua) {
    write_u64(out, q[0]);
    write_u64(out, q[1]);
    write_u64(out, q[2]);
    write_u64(out, q[3]);
  }
  for (const auto &te : tet) {
    write_u64(out, te[0]);
    write_u64(out, te[1]);
    write_u64(out, te[2]);
    write_u64(out, te[3]);
  }
  for (const auto &h : hex) {
    for (int i = 0; i < 8; ++i)
      write_u64(out, h[i]);
  }
  for (const auto &l : lin) {
    write_u64(out, l[0]);
    write_u64(out, l[1]);
  }

  return out;
}

boost::any decode_geometry(const std::vector<unsigned char> &bytes) {
  const unsigned char *p = bytes.data();
  const unsigned char *end = p + bytes.size();

  if (bytes.size() < 4 || std::memcmp(p, GEO_MAGIC, 4) != 0)
    throw std::runtime_error("decode_geometry: bad magic");
  p += 4;

  std::uint32_t gtype;
  std::uint64_t np, nn, nc, nt, nq, nte, nh, nl;
  if (!read_u32(p, end, gtype) || !read_u64(p, end, np) || !read_u64(p, end, nn) ||
      !read_u64(p, end, nc) || !read_u64(p, end, nt) || !read_u64(p, end, nq) ||
      !read_u64(p, end, nte) || !read_u64(p, end, nh) || !read_u64(p, end, nl))
    throw std::runtime_error("decode_geometry: truncated header");

  thread_local app decode_ctx;
  geometry geo(decode_ctx);
  geo.set_geometry_type(static_cast<geometry::geometry_type>(gtype));

  auto &pts = geo.points();
  pts.resize(np);
  for (std::uint64_t i = 0; i < np; ++i) {
    if (!read_f64(p, end, pts[i][0]) || !read_f64(p, end, pts[i][1]) ||
        !read_f64(p, end, pts[i][2]))
      throw std::runtime_error("decode_geometry: truncated points");
  }

  auto &nrm = geo.normals();
  nrm.resize(nn);
  for (std::uint64_t i = 0; i < nn; ++i) {
    if (!read_f64(p, end, nrm[i][0]) || !read_f64(p, end, nrm[i][1]) ||
        !read_f64(p, end, nrm[i][2]))
      throw std::runtime_error("decode_geometry: truncated normals");
  }

  auto &clr = geo.colors();
  clr.resize(nc);
  for (std::uint64_t i = 0; i < nc; ++i) {
    if (!read_f64(p, end, clr[i][0]) || !read_f64(p, end, clr[i][1]) ||
        !read_f64(p, end, clr[i][2]))
      throw std::runtime_error("decode_geometry: truncated colors");
  }

  auto &tri = geo.tris();
  tri.resize(nt);
  for (std::uint64_t i = 0; i < nt; ++i) {
    if (!read_u64(p, end, tri[i][0]) || !read_u64(p, end, tri[i][1]) ||
        !read_u64(p, end, tri[i][2]))
      throw std::runtime_error("decode_geometry: truncated tris");
  }

  auto &qua = geo.quads();
  qua.resize(nq);
  for (std::uint64_t i = 0; i < nq; ++i) {
    if (!read_u64(p, end, qua[i][0]) || !read_u64(p, end, qua[i][1]) ||
        !read_u64(p, end, qua[i][2]) || !read_u64(p, end, qua[i][3]))
      throw std::runtime_error("decode_geometry: truncated quads");
  }

  auto &tet = geo.tets();
  tet.resize(nte);
  for (std::uint64_t i = 0; i < nte; ++i) {
    if (!read_u64(p, end, tet[i][0]) || !read_u64(p, end, tet[i][1]) ||
        !read_u64(p, end, tet[i][2]) || !read_u64(p, end, tet[i][3]))
      throw std::runtime_error("decode_geometry: truncated tets");
  }

  auto &hex = geo.hexs();
  hex.resize(nh);
  for (std::uint64_t i = 0; i < nh; ++i) {
    for (int k = 0; k < 8; ++k) {
      if (!read_u64(p, end, hex[i][k]))
        throw std::runtime_error("decode_geometry: truncated hexs");
    }
  }

  auto &lin = geo.lines();
  lin.resize(nl);
  for (std::uint64_t i = 0; i < nl; ++i) {
    if (!read_u64(p, end, lin[i][0]) || !read_u64(p, end, lin[i][1]))
      throw std::runtime_error("decode_geometry: truncated lines");
  }

  return boost::any(geo);
}

} // anonymous namespace

void register_volume_geometry_codecs(state_codec_registry &registry) {
  registry.register_codec("cvc::volume", encode_volume, decode_volume, "cvc.volume.v1");
  registry.register_codec("cvc::geometry", encode_geometry, decode_geometry, "cvc.geometry.v1");
}

} // namespace cvc
