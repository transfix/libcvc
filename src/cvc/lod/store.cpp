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

// store.cpp -- scene.cvch5 LOD pyramid persistence (file + in-memory). HDF5 only.

#include <H5Cpp.h>
#include <algorithm>
#include <atomic>
#include <boost/make_shared.hpp>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/core/state_blob_store.h>
#include <cvc/lod/store.h>
#include <cvc/volume/hdf5_utils.h>
#include <fstream>
#include <sstream>

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

template <class T>
hsize_t read_2d(const Group &g, const char *name, hsize_t cols, std::vector<T> &out,
                const PredType &pt) {
  out.clear();
  if (!g.nameExists(name))
    return 0;
  DataSet ds = g.openDataSet(name);
  DataSpace sp = ds.getSpace();
  if (sp.getSimpleExtentNdims() != 2)
    throw hdf5_exception(std::string("lod::store: rank!=2 for ") + name);
  hsize_t dims[2] = {0, 0};
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

// ── per-asset ops on an open file (the caller already holds the library lock) ──

void write_mesh_into(H5File &f, const std::string &name, const mesh_pyramid &pyr,
                     const std::string &hash) {
  const std::string base = asset_path('M', name);
  try {
    f.unlink(base); // replace any prior pyramid at this name (no-op if absent)
  } catch (const H5::Exception &) {
  }
  boost::shared_ptr<Group> ag = hu::getGroup(f, base, true);
  hu::setAttribute<std::string>(*ag, "kind", std::string("M"));
  hu::setAttribute<int>(*ag, "nrungs", int(pyr.rungs.size()));
  hu::setAttribute<std::string>(*ag, "source_hash", hash);
  if (!pyr.world_error_m.empty())
    hu::setAttribute<double>(*ag, "world_error_m", pyr.world_error_m.size(),
                             pyr.world_error_m.data());
  for (std::size_t k = 0; k < pyr.rungs.size(); ++k) {
    const geometry &m = pyr.rungs[k];
    boost::shared_ptr<Group> rg = hu::getGroup(f, base + "/lod/" + std::to_string(k), true);
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

mesh_pyramid read_mesh_from(app &ctx, H5File &f, const std::string &name) {
  mesh_pyramid out;
  const std::string base = asset_path('M', name);
  if (!f.nameExists("/cvc/geometry") || !f.nameExists(base))
    throw hdf5_exception("lod::read_mesh_pyramid: no mesh pyramid '" + name + "'");
  boost::shared_ptr<Group> ag = hu::getGroup(f, base, false);
  int nrungs = 0;
  hu::getAttribute<int>(*ag, "nrungs", nrungs);
  for (int k = 0; k < nrungs; ++k) {
    Group rg = f.openGroup(base + "/lod/" + std::to_string(k));
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

void write_image_into(H5File &f, const std::string &name, const image_pyramid &pyr,
                      const std::string &hash) {
  const std::string base = asset_path('I', name);
  try {
    f.unlink(base);
  } catch (const H5::Exception &) {
  }
  boost::shared_ptr<Group> ag = hu::getGroup(f, base, true);
  hu::setAttribute<std::string>(*ag, "kind", std::string("I"));
  hu::setAttribute<int>(*ag, "nrungs", int(pyr.rungs.size()));
  hu::setAttribute<std::string>(*ag, "source_hash", hash);
  if (!pyr.world_error_m.empty())
    hu::setAttribute<double>(*ag, "world_error_m", pyr.world_error_m.size(),
                             pyr.world_error_m.data());
  for (std::size_t k = 0; k < pyr.rungs.size(); ++k) {
    const image &im = pyr.rungs[k];
    boost::shared_ptr<Group> rg = hu::getGroup(f, base + "/lod/" + std::to_string(k), true);
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

image_pyramid read_image_from(H5File &f, const std::string &name) {
  image_pyramid out;
  const std::string base = asset_path('I', name);
  if (!f.nameExists("/cvc/images") || !f.nameExists(base))
    throw hdf5_exception("lod::read_image_pyramid: no image pyramid '" + name + "'");
  boost::shared_ptr<Group> ag = hu::getGroup(f, base, false);
  int nrungs = 0;
  hu::getAttribute<int>(*ag, "nrungs", nrungs);
  for (int k = 0; k < nrungs; ++k) {
    Group rg = f.openGroup(base + "/lod/" + std::to_string(k));
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

std::vector<lod_index_entry> read_index_from(H5File &f) {
  std::vector<lod_index_entry> out;
  const char kinds[3] = {'M', 'I', 'V'};
  for (char kind : kinds) {
    std::string kg = std::string("/cvc/") + kind_group(kind);
    if (!f.nameExists("/cvc") || !f.nameExists(kg))
      continue;
    Group g = f.openGroup(kg);
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

// ── backings ──

// A name no other open HDF5 file in this process can have. HDF5 knows an open
// file only by what its driver compares, and the core VFD with no backing file
// compares NAMES (strcmp). A second open of a name already open is handed the
// first file's shared state: with one fixed name, a second live in-memory
// writer fails (HDF5 won't truncate an open file) and a second blob reader
// silently reads the FIRST blob's bytes. The atomic counter makes every name
// unique across threads; the counter's own address keeps two statically linked
// copies of libcvc that share one HDF5 apart. Nothing is created on disk
// (backing_store is off) -- the name is only an identity.
std::string unique_core_name(const char *what) {
  static std::atomic<std::uint64_t> counter(0);
  std::ostringstream os;
  os << "cvc-lod-" << what << '-' << static_cast<const void *>(&counter) << '-'
     << counter.fetch_add(1, std::memory_order_relaxed) << ".cvch5";
  return os.str();
}

boost::shared_ptr<H5File> open_memory(const std::string &name) {
  FileAccPropList fapl;
  fapl.setCore(64u * 1024u, /*backing_store=*/false);
  return boost::make_shared<H5File>(name, H5F_ACC_TRUNC, FileCreatPropList::DEFAULT, fapl);
}
boost::shared_ptr<H5File> open_blob(const std::string &name, const unsigned char *bytes,
                                    std::size_t n) {
  FileAccPropList fapl;
  fapl.setCore(64u * 1024u, /*backing_store=*/false);
  // Supply the in-RAM image so the "file" is exactly these bytes (no temp file).
  // HDF5 copies them, so the caller's buffer need not outlive the reader.
  if (H5Pset_file_image(fapl.getId(), const_cast<unsigned char *>(bytes), n) < 0)
    throw hdf5_exception("lod::store: H5Pset_file_image failed");
  return boost::make_shared<H5File>(name, H5F_ACC_RDONLY, FileCreatPropList::DEFAULT, fapl);
}
std::vector<unsigned char> file_image(H5File &f) {
  f.flush(H5F_SCOPE_GLOBAL);
  ssize_t sz = H5Fget_file_image(f.getId(), nullptr, 0);
  if (sz <= 0)
    return {};
  const std::size_t len = static_cast<std::size_t>(sz);
  std::vector<unsigned char> buf(len);
  if (H5Fget_file_image(f.getId(), buf.data(), len) < 0)
    throw hdf5_exception("lod::store: H5Fget_file_image failed");
  return buf;
}

} // namespace

std::string mesh_content_hash(const geometry &g) { return sha256_hex(mesh_bytes(g)); }
std::string image_content_hash(const image &img) {
  return sha256_hex(img.data(), img.size_bytes());
}

// ── scene_writer ──
struct scene_writer::impl {
  app &ctx;
  boost::shared_ptr<H5File> f;
  std::string key;
  impl(app &c, boost::shared_ptr<H5File> ff, std::string k) : ctx(c), f(ff), key(std::move(k)) {}
};

scene_writer::scene_writer(app &ctx) {
  const std::string name = unique_core_name("mem");
  hu::library_lock lock(ctx, name, "cvc::lod::scene_writer(memory)");
  H5::Exception::dontPrint();
  _p.reset(new impl(ctx, open_memory(name), name));
}
scene_writer::scene_writer(app &ctx, const std::string &path) {
  hu::library_lock lock(ctx, path, "cvc::lod::scene_writer(file)");
  H5::Exception::dontPrint();
  _p.reset(new impl(ctx, hu::getH5File(path, /*create-or-open, preserving*/ false), path));
}
scene_writer::~scene_writer() = default;
scene_writer::scene_writer(scene_writer &&) noexcept = default;
scene_writer &scene_writer::operator=(scene_writer &&) noexcept = default;

void scene_writer::write_mesh_pyramid(const std::string &name, const mesh_pyramid &pyr,
                                      const std::string &source_hash) {
  hu::library_lock lock(_p->ctx, _p->key, "cvc::lod::scene_writer::write_mesh_pyramid");
  H5::Exception::dontPrint();
  write_mesh_into(*_p->f, name, pyr, source_hash);
}
void scene_writer::write_image_pyramid(const std::string &name, const image_pyramid &pyr,
                                       const std::string &source_hash) {
  hu::library_lock lock(_p->ctx, _p->key, "cvc::lod::scene_writer::write_image_pyramid");
  H5::Exception::dontPrint();
  write_image_into(*_p->f, name, pyr, source_hash);
}
std::vector<unsigned char> scene_writer::to_blob() {
  hu::library_lock lock(_p->ctx, _p->key, "cvc::lod::scene_writer::to_blob");
  H5::Exception::dontPrint();
  return file_image(*_p->f);
}

// ── scene_reader ──
struct scene_reader::impl {
  app &ctx;
  boost::shared_ptr<H5File> f;
  std::string key;
  impl(app &c, boost::shared_ptr<H5File> ff, std::string k) : ctx(c), f(ff), key(std::move(k)) {}
};

scene_reader::scene_reader(app &ctx, const std::string &path) {
  hu::library_lock lock(ctx, path, "cvc::lod::scene_reader(file)");
  H5::Exception::dontPrint();
  _p.reset(new impl(ctx, hu::getH5File(path, false), path));
}
scene_reader::scene_reader(app &ctx, const unsigned char *bytes, std::size_t n) {
  const std::string name = unique_core_name("blob");
  hu::library_lock lock(ctx, name, "cvc::lod::scene_reader(blob)");
  H5::Exception::dontPrint();
  _p.reset(new impl(ctx, open_blob(name, bytes, n), name));
}
scene_reader::~scene_reader() = default;
scene_reader::scene_reader(scene_reader &&) noexcept = default;
scene_reader &scene_reader::operator=(scene_reader &&) noexcept = default;

mesh_pyramid scene_reader::read_mesh_pyramid(const std::string &name) {
  hu::library_lock lock(_p->ctx, _p->key, "cvc::lod::scene_reader::read_mesh_pyramid");
  H5::Exception::dontPrint();
  return read_mesh_from(_p->ctx, *_p->f, name);
}
image_pyramid scene_reader::read_image_pyramid(const std::string &name) {
  hu::library_lock lock(_p->ctx, _p->key, "cvc::lod::scene_reader::read_image_pyramid");
  H5::Exception::dontPrint();
  return read_image_from(*_p->f, name);
}
std::vector<lod_index_entry> scene_reader::index() {
  hu::library_lock lock(_p->ctx, _p->key, "cvc::lod::scene_reader::index");
  H5::Exception::dontPrint();
  return read_index_from(*_p->f);
}
bool scene_reader::has(const std::string &name, const std::string &source_hash) {
  try {
    for (const auto &e : index())
      if (e.name == name)
        return source_hash.empty() || e.source_hash == source_hash;
  } catch (...) {
  }
  return false;
}

// ── free functions (file only) ──
void write_mesh_pyramid(app &ctx, const std::string &h5file, const std::string &name,
                        const mesh_pyramid &pyr, const std::string &source_hash) {
  scene_writer(ctx, h5file).write_mesh_pyramid(name, pyr, source_hash);
}
mesh_pyramid read_mesh_pyramid(app &ctx, const std::string &h5file, const std::string &name) {
  return scene_reader(ctx, h5file).read_mesh_pyramid(name);
}
void write_image_pyramid(app &ctx, const std::string &h5file, const std::string &name,
                         const image_pyramid &pyr, const std::string &source_hash) {
  scene_writer(ctx, h5file).write_image_pyramid(name, pyr, source_hash);
}
image_pyramid read_image_pyramid(app &ctx, const std::string &h5file, const std::string &name) {
  return scene_reader(ctx, h5file).read_image_pyramid(name);
}
std::vector<lod_index_entry> read_lod_index(app &ctx, const std::string &h5file) {
  return scene_reader(ctx, h5file).index();
}
bool has_pyramid(app &ctx, const std::string &h5file, const std::string &name,
                 const std::string &source_hash) {
  try {
    return scene_reader(ctx, h5file).has(name, source_hash);
  } catch (...) {
    return false;
  }
}

bool bake_mesh_asset(app &ctx, const std::string &h5file, const std::string &name,
                     const geometry &src, const pyramid_params &params, bool force,
                     thread_pool *pool) {
  const std::string hash = mesh_content_hash(src);
  if (!force) {
    std::ifstream probe(h5file.c_str(), std::ios::binary);
    if (probe.good()) {
      probe.close();
      try {
        if (scene_reader(ctx, h5file).has(name, hash))
          return false; // an up-to-date pyramid is already baked
      } catch (...) {
      }
    }
  }
  mesh_pyramid pyr = build_mesh_pyramid(src, params, pool);
  scene_writer(ctx, h5file).write_mesh_pyramid(name, pyr, hash);
  return true;
}

} // namespace lod
} // namespace cvc
