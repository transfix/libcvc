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
#include <memory>
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
// H5Lexists FAILS, rather than answering false, when an intermediate group is
// missing (e.g. "/cvc/geometry/x" in a container with no /cvc yet), so walk the
// path one component at a time.
bool path_exists(const H5File &f, const std::string &path) {
  for (std::size_t pos = path.find('/', 1); pos != std::string::npos; pos = path.find('/', pos + 1))
    if (!f.nameExists(path.substr(0, pos)))
      return false;
  return f.nameExists(path);
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
  if (!path_exists(f, base))
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
  if (!path_exists(f, base))
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
    if (fmt < int(image::pixel_format::GRAY) || fmt > int(image::pixel_format::RGBA))
      throw hdf5_exception("lod::read_image_pyramid: bad pixel format in '" + name + "'");
    image im(w, h, image::pixel_format(fmt), image::data_type::u8);
    DataSet ds = rg.openDataSet("pixels");
    // The read fills the whole dataset into im's buffer, so the dataset must be
    // exactly the H x W x C the attributes sized it for -- a blob is untrusted
    // input, and a mismatch would otherwise overrun the image.
    DataSpace sp = ds.getSpace();
    hsize_t dims[3] = {0, 0, 0};
    const bool rank3 = sp.getSimpleExtentNdims() == 3;
    if (rank3)
      sp.getSimpleExtentDims(dims);
    if (!rank3 || dims[0] != hsize_t(std::max(h, 0)) || dims[1] != hsize_t(std::max(w, 0)) ||
        dims[2] != hsize_t(im.channels()))
      throw hdf5_exception("lod::read_image_pyramid: pixels do not match w/h/format in '" + name +
                           "'");
    if (im.size_bytes())
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
    if (!path_exists(f, kg))
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

// ── error boundary ──
// H5::Exception does not derive from std::exception, so one escaping a public
// call slips past every `catch (const std::exception &)` (cvc-lod-bake's
// included). Each public entry point runs its HDF5 work through guarded(), which
// rethrows it as cvc::hdf5_exception -- what the rest of hdf5_utils reports --
// naming the call, the file and HDF5's own reason. Both run with the library
// lock held: reading the error stack is itself an HDF5 call.

herr_t innermost_error(unsigned n, const H5E_error2_t *err, void *out) {
  if (n == 0 && err && err->desc)
    *static_cast<std::string *>(out) = err->desc;
  return 0;
}

hdf5_exception h5_failure(const char *call, const std::string &file, const H5::Exception &e) {
  // The wrapper's text ("H5Fopen failed") says little; the innermost entry of
  // HDF5's error stack ("file signature not found") says why. Any later HDF5
  // call -- a handle closed while unwinding -- clears that stack, so this is
  // best-effort unless called right at the failure.
  std::string cause;
  H5Ewalk2(H5E_DEFAULT, H5E_WALK_UPWARD, innermost_error, &cause);
  std::string msg =
      std::string(call) + " '" + file + "': " + e.getFuncName() + ": " + e.getDetailMsg();
  if (!cause.empty())
    msg += " (" + cause + ")";
  return hdf5_exception(msg);
}

template <class F>
auto guarded(const char *call, const std::string &file, F &&fn) -> decltype(fn()) {
  try {
    return fn();
  } catch (const H5::Exception &e) {
    throw h5_failure(call, file, e);
  }
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

// The core-VFD opens report their own failure: closing `fapl` on the way out
// would clear HDF5's error stack before guarded() could read it.
boost::shared_ptr<H5File> open_memory(const std::string &name) {
  FileAccPropList fapl;
  fapl.setCore(64u * 1024u, /*backing_store=*/false);
  try {
    return boost::make_shared<H5File>(name, H5F_ACC_TRUNC, FileCreatPropList::DEFAULT, fapl);
  } catch (const H5::Exception &e) {
    throw h5_failure("lod::scene_writer", name, e);
  }
}
boost::shared_ptr<H5File> open_blob(const std::string &name, const unsigned char *bytes,
                                    std::size_t n) {
  if (!bytes || n == 0)
    throw hdf5_exception("lod::scene_reader: empty blob");
  FileAccPropList fapl;
  fapl.setCore(64u * 1024u, /*backing_store=*/false);
  // Supply the in-RAM image so the "file" is exactly these bytes (no temp file).
  // HDF5 copies them, so the caller's buffer need not outlive the reader.
  if (H5Pset_file_image(fapl.getId(), const_cast<unsigned char *>(bytes), n) < 0)
    throw hdf5_exception("lod::store: H5Pset_file_image failed");
  try {
    return boost::make_shared<H5File>(name, H5F_ACC_RDONLY, FileCreatPropList::DEFAULT, fapl);
  } catch (const H5::Exception &e) {
    throw h5_failure("lod::scene_reader", name, e);
  }
}
// A reader opens an EXISTING file read-only. (hdf5_utils::getH5File is
// create-or-open: it would create a missing path and truncate a non-HDF5 one.)
boost::shared_ptr<H5File> open_existing(const std::string &path) {
  return boost::make_shared<H5File>(path, H5F_ACC_RDONLY);
}
// A writer creates or opens, preserving prior assets. getH5File quietly falls
// back to read-only when it cannot get write access; say so now rather than
// fail at the first write with an opaque "H5Gcreate2 failed".
boost::shared_ptr<H5File> open_writable(const std::string &path) {
  boost::shared_ptr<H5File> f = hu::getH5File(path, false);
  unsigned intent = 0;
  if (H5Fget_intent(f->getId(), &intent) < 0 || !(intent & H5F_ACC_RDWR))
    throw hdf5_exception("lod::scene_writer '" + path +
                         "': opened read-only (not writable, or a scene_reader has it open)");
  return f;
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

// Destroys `p` -- closing its HDF5 file -- under the library lock, since
// H5Fclose is an entry into HDF5 like any other (hdf5_utils.h, library_lock).
// Never throws: were the lock ever unobtainable, the handle is leaked rather
// than closed unlocked, and HDF5 closes it at exit.
template <class Impl> void release_locked(std::unique_ptr<Impl> &p) noexcept {
  if (!p)
    return; // moved-from
  try {
    hu::library_lock lock(p->ctx, p->key, "cvc::lod::store close");
    p.reset();
  } catch (...) {
    static_cast<void>(p.release());
  }
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
  _p.reset(
      new impl(ctx, guarded("lod::scene_writer", name, [&] { return open_memory(name); }), name));
}
scene_writer::scene_writer(app &ctx, const std::string &path) {
  hu::library_lock lock(ctx, path, "cvc::lod::scene_writer(file)");
  H5::Exception::dontPrint();
  _p.reset(
      new impl(ctx, guarded("lod::scene_writer", path, [&] { return open_writable(path); }), path));
}
scene_writer::~scene_writer() { release_locked(_p); }
scene_writer::scene_writer(scene_writer &&) noexcept = default;
scene_writer &scene_writer::operator=(scene_writer &&o) noexcept {
  if (this != &o) {
    release_locked(_p); // a defaulted move would close the old file unlocked
    _p = std::move(o._p);
  }
  return *this;
}

void scene_writer::write_mesh_pyramid(const std::string &name, const mesh_pyramid &pyr,
                                      const std::string &source_hash) {
  hu::library_lock lock(_p->ctx, _p->key, "cvc::lod::scene_writer::write_mesh_pyramid");
  H5::Exception::dontPrint();
  guarded("lod::scene_writer::write_mesh_pyramid", _p->key,
          [&] { write_mesh_into(*_p->f, name, pyr, source_hash); });
}
void scene_writer::write_image_pyramid(const std::string &name, const image_pyramid &pyr,
                                       const std::string &source_hash) {
  hu::library_lock lock(_p->ctx, _p->key, "cvc::lod::scene_writer::write_image_pyramid");
  H5::Exception::dontPrint();
  guarded("lod::scene_writer::write_image_pyramid", _p->key,
          [&] { write_image_into(*_p->f, name, pyr, source_hash); });
}
std::vector<unsigned char> scene_writer::to_blob() {
  hu::library_lock lock(_p->ctx, _p->key, "cvc::lod::scene_writer::to_blob");
  H5::Exception::dontPrint();
  return guarded("lod::scene_writer::to_blob", _p->key, [&] { return file_image(*_p->f); });
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
  _p.reset(
      new impl(ctx, guarded("lod::scene_reader", path, [&] { return open_existing(path); }), path));
}
scene_reader::scene_reader(app &ctx, const unsigned char *bytes, std::size_t n) {
  const std::string name = unique_core_name("blob");
  hu::library_lock lock(ctx, name, "cvc::lod::scene_reader(blob)");
  H5::Exception::dontPrint();
  _p.reset(new impl(
      ctx, guarded("lod::scene_reader", name, [&] { return open_blob(name, bytes, n); }), name));
}
scene_reader::~scene_reader() { release_locked(_p); }
scene_reader::scene_reader(scene_reader &&) noexcept = default;
scene_reader &scene_reader::operator=(scene_reader &&o) noexcept {
  if (this != &o) {
    release_locked(_p); // a defaulted move would close the old file unlocked
    _p = std::move(o._p);
  }
  return *this;
}

mesh_pyramid scene_reader::read_mesh_pyramid(const std::string &name) {
  hu::library_lock lock(_p->ctx, _p->key, "cvc::lod::scene_reader::read_mesh_pyramid");
  H5::Exception::dontPrint();
  return guarded("lod::scene_reader::read_mesh_pyramid", _p->key,
                 [&] { return read_mesh_from(_p->ctx, *_p->f, name); });
}
image_pyramid scene_reader::read_image_pyramid(const std::string &name) {
  hu::library_lock lock(_p->ctx, _p->key, "cvc::lod::scene_reader::read_image_pyramid");
  H5::Exception::dontPrint();
  return guarded("lod::scene_reader::read_image_pyramid", _p->key,
                 [&] { return read_image_from(*_p->f, name); });
}
std::vector<lod_index_entry> scene_reader::index() {
  hu::library_lock lock(_p->ctx, _p->key, "cvc::lod::scene_reader::index");
  H5::Exception::dontPrint();
  return guarded("lod::scene_reader::index", _p->key, [&] { return read_index_from(*_p->f); });
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
    try {
      // A missing or unreadable h5file throws here (the reader never creates
      // one) and simply means there is nothing current to skip.
      if (scene_reader(ctx, h5file).has(name, hash))
        return false; // an up-to-date pyramid is already baked
    } catch (...) {
    }
  }
  mesh_pyramid pyr = build_mesh_pyramid(src, params, pool);
  scene_writer(ctx, h5file).write_mesh_pyramid(name, pyr, hash);
  return true;
}

} // namespace lod
} // namespace cvc
