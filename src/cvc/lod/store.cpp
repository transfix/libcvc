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

// store.cpp -- scene.cvch5 LOD pyramid persistence. Compiled only with HDF5.

#include <H5Cpp.h>
#include <algorithm>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/core/state_blob_store.h>
#include <cvc/lod/store.h>
#include <cvc/volume/hdf5_utils.h>

namespace cvc {
namespace lod {

using namespace H5;
namespace hu = cvc::hdf5_utils;

namespace {

const char *kind_group(char kind) {
  switch (kind) {
  case 'M':
    return "geometry";
  case 'I':
    return "images";
  case 'V':
    return "volumes";
  }
  return "geometry";
}
std::string asset_path(char kind, const std::string &name) {
  return std::string("/cvc/") + kind_group(kind) + "/" + name;
}

// A contiguous typed 2-D array (rows x cols); libcvc's point/tri/uv/color arrays
// are all std::vector<boost::array<T,cols>>, so their storage is exactly this.
template <class T>
void write_2d(Group &g, const char *name, const T *data, hsize_t rows, hsize_t cols,
              const PredType &pt) {
  if (rows == 0)
    return;
  hsize_t dims[2] = {rows, cols};
  DataSpace sp(2, dims);
  DataSet ds = g.createDataSet(name, pt, sp);
  ds.write(data, pt);
}

// Read an (rows x cols) dataset into `out` (resized to rows*cols); returns rows.
// cols is asserted. Missing dataset -> out cleared, returns 0.
template <class T>
hsize_t read_2d(const Group &g, const char *name, hsize_t cols, std::vector<T> &out,
                const PredType &pt) {
  out.clear();
  if (!g.nameExists(name))
    return 0;
  DataSet ds = g.openDataSet(name);
  DataSpace sp = ds.getSpace();
  hsize_t dims[2] = {0, 0};
  if (sp.getSimpleExtentNdims() != 2)
    throw hdf5_exception(std::string("lod::store: rank!=2 for ") + name);
  sp.getSimpleExtentDims(dims);
  if (dims[1] != cols)
    throw hdf5_exception(std::string("lod::store: cols mismatch for ") + name);
  out.resize(std::size_t(dims[0]) * cols);
  if (dims[0])
    ds.read(out.data(), pt);
  return dims[0];
}

std::vector<unsigned char> mesh_bytes(const geometry &g) {
  const geometry::points_t &P = g.const_points();
  const geometry::tris_t &T = g.const_tris();
  std::vector<unsigned char> b;
  b.reserve(P.size() * sizeof(geometry::point_t) + T.size() * sizeof(geometry::tri_t));
  auto add = [&](const void *p, std::size_t n) {
    const unsigned char *c = static_cast<const unsigned char *>(p);
    b.insert(b.end(), c, c + n);
  };
  if (!P.empty())
    add(P.data(), P.size() * sizeof(geometry::point_t));
  if (!T.empty())
    add(T.data(), T.size() * sizeof(geometry::tri_t));
  return b;
}

} // namespace

std::string mesh_content_hash(const geometry &g) { return sha256_hex(mesh_bytes(g)); }

std::string image_content_hash(const image &img) {
  return sha256_hex(img.data(), img.size_bytes());
}

void write_mesh_pyramid(app &ctx, const std::string &h5file, const std::string &name,
                        const mesh_pyramid &pyr, const std::string &source_hash) {
  hu::library_lock lock(ctx, h5file, "cvc::lod::write_mesh_pyramid");
  H5::Exception::dontPrint();
  boost::shared_ptr<H5File> f = hu::getH5File(h5file, /*create-or-open, preserving*/ false);
  const std::string base = asset_path('M', name);
  try {
    f->unlink(base); // replace any prior pyramid at this name (no-op if absent)
  } catch (const H5::Exception &) {
  }
  boost::shared_ptr<Group> ag = hu::getGroup(*f, base, /*create=*/true);
  hu::setAttribute<std::string>(*ag, "kind", std::string("M"));
  hu::setAttribute<int>(*ag, "nrungs", int(pyr.rungs.size()));
  hu::setAttribute<std::string>(*ag, "source_hash", source_hash);
  if (!pyr.world_error_m.empty())
    hu::setAttribute<double>(*ag, "world_error_m", pyr.world_error_m.size(),
                             pyr.world_error_m.data());

  for (std::size_t k = 0; k < pyr.rungs.size(); ++k) {
    const geometry &m = pyr.rungs[k];
    boost::shared_ptr<Group> rg =
        hu::getGroup(*f, base + "/lod/" + std::to_string(k), /*create=*/true);
    const geometry::points_t &P = m.const_points();
    const geometry::tris_t &T = m.const_tris();
    write_2d(*rg, "points", P.empty() ? nullptr : &P[0][0], P.size(), 3, PredType::NATIVE_DOUBLE);
    write_2d(*rg, "tris", T.empty() ? nullptr : &T[0][0], T.size(), 3, PredType::NATIVE_UINT64);
    const geometry::uvs_t &U = m.const_uvs();
    if (U.size() == P.size() && !U.empty())
      write_2d(*rg, "uv", &U[0][0], U.size(), 2, PredType::NATIVE_DOUBLE);
    const geometry::colors_t &C = m.const_colors();
    if (C.size() == P.size() && !C.empty())
      write_2d(*rg, "colors", &C[0][0], C.size(), 3, PredType::NATIVE_DOUBLE);
    hu::setAttribute<double>(*rg, "world_error_m",
                             k < pyr.world_error_m.size() ? pyr.world_error_m[k] : 0.0);
    hu::setAttribute<cvc::uint64>(*rg, "ntris", cvc::uint64(T.size()));
  }
}

mesh_pyramid read_mesh_pyramid(app &ctx, const std::string &h5file, const std::string &name) {
  hu::library_lock lock(ctx, h5file, "cvc::lod::read_mesh_pyramid");
  H5::Exception::dontPrint();
  mesh_pyramid out;
  boost::shared_ptr<H5File> f = hu::getH5File(h5file, false);
  const std::string base = asset_path('M', name);
  if (!f->nameExists(base))
    throw hdf5_exception("lod::read_mesh_pyramid: no mesh pyramid '" + name + "'");
  boost::shared_ptr<Group> ag = hu::getGroup(*f, base, false);
  int nrungs = 0;
  hu::getAttribute<int>(*ag, "nrungs", nrungs);
  for (int k = 0; k < nrungs; ++k) {
    Group rg = f->openGroup(base + "/lod/" + std::to_string(k));
    geometry m(ctx);
    std::vector<double> P;
    hsize_t nv = read_2d(rg, "points", 3, P, PredType::NATIVE_DOUBLE);
    std::vector<std::uint64_t> Tt;
    hsize_t nt = read_2d(rg, "tris", 3, Tt, PredType::NATIVE_UINT64);
    geometry::points_t &mp = m.points();
    mp.resize(nv);
    for (hsize_t i = 0; i < nv; ++i)
      mp[i] = {P[i * 3], P[i * 3 + 1], P[i * 3 + 2]};
    geometry::tris_t &mt = m.tris();
    mt.resize(nt);
    for (hsize_t i = 0; i < nt; ++i)
      mt[i] = {Tt[i * 3], Tt[i * 3 + 1], Tt[i * 3 + 2]};
    std::vector<double> U;
    hsize_t nu = read_2d(rg, "uv", 2, U, PredType::NATIVE_DOUBLE);
    if (nu == nv && nv) {
      geometry::uvs_t &mu = m.uvs();
      mu.resize(nv);
      for (hsize_t i = 0; i < nv; ++i)
        mu[i] = {U[i * 2], U[i * 2 + 1]};
    }
    std::vector<double> Cc;
    hsize_t nc = read_2d(rg, "colors", 3, Cc, PredType::NATIVE_DOUBLE);
    if (nc == nv && nv) {
      geometry::colors_t &mc = m.colors();
      mc.resize(nv);
      for (hsize_t i = 0; i < nv; ++i)
        mc[i] = {Cc[i * 3], Cc[i * 3 + 1], Cc[i * 3 + 2]};
    }
    m.set_geometry_type(geometry::SURFACE_TRI);
    double we = 0.0;
    hu::getAttribute<double>(rg, "world_error_m", we);
    out.rungs.push_back(std::move(m));
    out.world_error_m.push_back(we);
  }
  return out;
}

void write_image_pyramid(app &ctx, const std::string &h5file, const std::string &name,
                         const image_pyramid &pyr, const std::string &source_hash) {
  hu::library_lock lock(ctx, h5file, "cvc::lod::write_image_pyramid");
  H5::Exception::dontPrint();
  boost::shared_ptr<H5File> f = hu::getH5File(h5file, /*create-or-open, preserving*/ false);
  const std::string base = asset_path('I', name);
  try {
    f->unlink(base); // replace any prior pyramid at this name (no-op if absent)
  } catch (const H5::Exception &) {
  }
  boost::shared_ptr<Group> ag = hu::getGroup(*f, base, true);
  hu::setAttribute<std::string>(*ag, "kind", std::string("I"));
  hu::setAttribute<int>(*ag, "nrungs", int(pyr.rungs.size()));
  hu::setAttribute<std::string>(*ag, "source_hash", source_hash);
  if (!pyr.world_error_m.empty())
    hu::setAttribute<double>(*ag, "world_error_m", pyr.world_error_m.size(),
                             pyr.world_error_m.data());
  for (std::size_t k = 0; k < pyr.rungs.size(); ++k) {
    const image &im = pyr.rungs[k];
    boost::shared_ptr<Group> rg = hu::getGroup(*f, base + "/lod/" + std::to_string(k), true);
    hsize_t dims[3] = {hsize_t(im.height()), hsize_t(im.width()), hsize_t(im.channels())};
    DataSpace sp(3, dims);
    DataSet ds = rg->createDataSet("pixels", PredType::NATIVE_UINT8, sp);
    ds.write(im.data(), PredType::NATIVE_UINT8);
    hu::setAttribute<int>(*rg, "w", im.width());
    hu::setAttribute<int>(*rg, "h", im.height());
    hu::setAttribute<int>(*rg, "channels", im.channels());
    hu::setAttribute<int>(*rg, "format", int(im.format()));
    hu::setAttribute<double>(*rg, "world_error_m",
                             k < pyr.world_error_m.size() ? pyr.world_error_m[k] : 0.0);
  }
}

image_pyramid read_image_pyramid(app &ctx, const std::string &h5file, const std::string &name) {
  hu::library_lock lock(ctx, h5file, "cvc::lod::read_image_pyramid");
  H5::Exception::dontPrint();
  image_pyramid out;
  boost::shared_ptr<H5File> f = hu::getH5File(h5file, false);
  const std::string base = asset_path('I', name);
  if (!f->nameExists(base))
    throw hdf5_exception("lod::read_image_pyramid: no image pyramid '" + name + "'");
  boost::shared_ptr<Group> ag = hu::getGroup(*f, base, false);
  int nrungs = 0;
  hu::getAttribute<int>(*ag, "nrungs", nrungs);
  for (int k = 0; k < nrungs; ++k) {
    Group rg = f->openGroup(base + "/lod/" + std::to_string(k));
    int w = 0, h = 0, fmt = int(image::pixel_format::RGBA);
    hu::getAttribute<int>(rg, "w", w);
    hu::getAttribute<int>(rg, "h", h);
    hu::getAttribute<int>(rg, "format", fmt);
    image im(w, h, image::pixel_format(fmt), image::data_type::u8);
    DataSet ds = rg.openDataSet("pixels");
    ds.read(im.data(), PredType::NATIVE_UINT8);
    double we = 0.0;
    hu::getAttribute<double>(rg, "world_error_m", we);
    out.rungs.push_back(std::move(im));
    out.world_error_m.push_back(we);
  }
  return out;
}

std::vector<lod_index_entry> read_lod_index(app &ctx, const std::string &h5file) {
  hu::library_lock lock(ctx, h5file, "cvc::lod::read_lod_index");
  H5::Exception::dontPrint();
  std::vector<lod_index_entry> out;
  boost::shared_ptr<H5File> f = hu::getH5File(h5file, false);
  const char kinds[3] = {'M', 'I', 'V'};
  for (char kind : kinds) {
    std::string kg = std::string("/cvc/") + kind_group(kind);
    if (!f->nameExists(kg))
      continue;
    Group g = f->openGroup(kg);
    const hsize_t n = g.getNumObjs();
    for (hsize_t i = 0; i < n; ++i) {
      std::string name = g.getObjnameByIdx(i);
      Group ag = g.openGroup(name);
      lod_index_entry e;
      e.name = name;
      e.kind = kind;
      hu::getAttribute<int>(ag, "nrungs", e.nrungs);
      try {
        hu::getAttribute<std::string>(ag, "source_hash", e.source_hash);
      } catch (...) {
      }
      if (ag.attrExists("world_error_m") && e.nrungs > 0) {
        e.world_error_m.resize(e.nrungs);
        hu::getAttribute<double>(ag, "world_error_m", e.nrungs, e.world_error_m.data());
      }
      out.push_back(std::move(e));
    }
  }
  return out;
}

bool has_pyramid(app &ctx, const std::string &h5file, const std::string &name,
                 const std::string &source_hash) {
  try {
    std::vector<lod_index_entry> idx = read_lod_index(ctx, h5file);
    for (const auto &e : idx)
      if (e.name == name)
        return source_hash.empty() || e.source_hash == source_hash;
  } catch (...) {
  }
  return false;
}

} // namespace lod
} // namespace cvc
