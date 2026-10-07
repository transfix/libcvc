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
#include <cctype>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/lod/store.h>
#include <cvc/state/state_blob_store.h>
#include <cvc/volume/hdf5_utils.h>
#include <filesystem>
#include <limits>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <tuple>

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

// The innermost entry of HDF5's error stack ("file signature not found") says
// why a call failed; the wrappers above it say little. Read it right at the
// failure: any later HDF5 call clears the stack.
herr_t innermost_error(unsigned n, const H5E_error2_t *err, void *out) {
  if (n == 0 && err && err->desc)
    *static_cast<std::string *>(out) = err->desc;
  return 0;
}
[[noreturn]] void h5_fail(const std::string &what) {
  std::string cause;
  H5Ewalk2(H5E_DEFAULT, H5E_WALK_UPWARD, innermost_error, &cause);
  throw hdf5_exception("lod::store: " + what + " failed" +
                       (cause.empty() ? "" : " (" + cause + ")"));
}

// ── reading a container that may be hostile (store.h, trust model) ──────────

// x * y, saturating instead of wrapping.
hsize_t mul_sat(hsize_t x, hsize_t y) {
  const hsize_t top = std::numeric_limits<hsize_t>::max();
  return (y != 0 && x > top / y) ? top : x * y;
}

// Every link the reader follows is checked to be a hard link first, so this
// never runs; it makes HDF5 itself refuse to traverse an external link too,
// should a path ever reach one.
herr_t deny_external_link(const char *, const char *, const char *, const char *, unsigned *, hid_t,
                          void *) {
  return -1;
}

// An access property list of class `cls` (link, group or dataset access) that
// denies external-link traversal. The C++ API has no group-access class, so all
// three are plain ids.
class access_plist {
public:
  explicit access_plist(hid_t cls) : _id(H5Pcreate(cls)) {
    if (_id < 0)
      h5_fail("H5Pcreate");
    if (H5Pset_elink_cb(_id, deny_external_link, nullptr) < 0) {
      H5Pclose(_id);
      h5_fail("H5Pset_elink_cb");
    }
  }
  ~access_plist() { H5Pclose(_id); }
  access_plist(const access_plist &) = delete;
  access_plist &operator=(const access_plist &) = delete;
  hid_t id() const { return _id; }

private:
  hid_t _id;
};

// Wraps a freshly opened id in its C++ object. The wrapper takes a reference
// of its own, so the one the open returned is dropped once it holds it.
template <class Obj> Obj adopt(hid_t id, const std::string &what) {
  if (id < 0)
    h5_fail(what);
  struct drop {
    hid_t id;
    ~drop() { H5Idec_ref(id); }
  } d{id};
  return Obj(id);
}

// Where an object's header lives in the file: two names for one object (hard
// links) share it.
haddr_t object_address(hid_t id) {
#if H5_VERSION_GE(1, 12, 0)
  H5O_info2_t oi;
  haddr_t addr = HADDR_UNDEF;
  if (H5Oget_info3(id, &oi, H5O_INFO_BASIC) < 0 ||
      H5VLnative_token_to_addr(id, oi.token, &addr) < 0)
    h5_fail("H5Oget_info");
  return addr;
#elif H5_VERSION_GE(1, 10, 3)
  H5O_info_t oi;
  if (H5Oget_info2(id, &oi, H5O_INFO_BASIC) < 0)
    h5_fail("H5Oget_info");
  return oi.addr;
#else
  H5O_info_t oi;
  if (H5Oget_info(id, &oi) < 0)
    h5_fail("H5Oget_info");
  return oi.addr;
#endif
}

const char *link_kind(H5L_type_t t) {
  switch (t) {
  case H5L_TYPE_SOFT:
    return "a soft link";
  case H5L_TYPE_EXTERNAL:
    return "an external link (into another file)";
  default:
    return "a user-defined link";
  }
}

// One public read call on an open container, made with the library lock held:
// the access property lists every open goes through, the objects the call has
// reached, and its byte budget (store.h, trust model). The stored bytes it
// reads may total the container's size -- distinct objects of a well-formed
// container occupy distinct bytes -- and the bytes decoded from them 8x that,
// the widest conversion a reader makes (a 1-byte element into an 8-byte
// double/uint64). Anything more reads some bytes twice, which is how a small
// crafted container poses as a huge one.
class reader_call {
public:
  reader_call(H5File &f, hsize_t container_bytes)
      : _f(f), _lapl(H5P_LINK_ACCESS), _gapl(H5P_GROUP_ACCESS), _dapl(H5P_DATASET_ACCESS),
        _bytes(container_bytes) {}

  hsize_t container_bytes() const { return _bytes; }

  // Whether `parent` has a link called `name` -- ONE link, not a path. A link
  // that is not a hard link is refused rather than followed or skipped.
  bool has(const Group &parent, const std::string &name) {
    if (name.empty() || name == "." || name.find('/') != std::string::npos)
      throw hdf5_exception("lod::store: '" + name + "' is not a link name");
    const htri_t exists = H5Lexists(parent.getId(), name.c_str(), _lapl.id());
    if (exists < 0)
      h5_fail("H5Lexists '" + name + "'");
    if (exists == 0)
      return false;
    H5L_info_t li;
    if (H5Lget_info(parent.getId(), name.c_str(), &li, _lapl.id()) < 0)
      h5_fail("H5Lget_info '" + name + "'");
    if (li.type != H5L_TYPE_HARD)
      throw hdf5_exception("lod::store: '" + name + "' is " + link_kind(li.type) +
                           "; a scene.cvch5 holds only hard links");
    return true;
  }

  // The group at absolute `path`, walked from the root one hard link at a
  // time; false if some component is missing.
  bool find_group(const std::string &path, Group &out) {
    Group g = adopt<Group>(H5Gopen2(_f.getId(), "/", _gapl.id()), "H5Gopen2 '/'");
    for (std::size_t pos = 0; pos < path.size();) {
      std::size_t end = path.find('/', pos);
      if (end == std::string::npos)
        end = path.size();
      const std::string link = path.substr(pos, end - pos);
      pos = end + 1;
      if (link.empty())
        continue;
      if (!has(g, link))
        return false;
      g = adopt<Group>(H5Gopen2(g.getId(), link.c_str(), _gapl.id()), "H5Gopen2 '" + link + "'");
    }
    out = g;
    return true;
  }

  // The group `name` in `parent`, which must be there; reached() once.
  Group group(const Group &parent, const std::string &name) {
    if (!has(parent, name))
      throw hdf5_exception("lod::store: no group '" + name + "'");
    Group g =
        adopt<Group>(H5Gopen2(parent.getId(), name.c_str(), _gapl.id()), "H5Gopen2 '" + name + "'");
    reached(g, name);
    return g;
  }

  // The dataset `name` in `parent`, which must be there; reached() once, and
  // refused -- before anything reads it -- unless its data lives inside this
  // container the way the store writes it. External raw storage would read
  // another file's bytes; a virtual layout maps other files; a chunked one
  // (with or without filters) is never written by the store.
  DataSet dataset(const Group &parent, const std::string &name) {
    if (!has(parent, name))
      throw hdf5_exception("lod::store: no dataset '" + name + "'");
    DataSet ds = adopt<DataSet>(H5Dopen2(parent.getId(), name.c_str(), _dapl.id()),
                                "H5Dopen2 '" + name + "'");
    reached(ds, name);
    const DSetCreatPropList dcpl = ds.getCreatePlist();
    if (dcpl.getExternalCount() > 0)
      throw hdf5_exception("lod::store: dataset '" + name +
                           "' keeps its data in an external file; a scene.cvch5 is self-contained");
    const H5D_layout_t layout = dcpl.getLayout();
    if (layout != H5D_CONTIGUOUS && layout != H5D_COMPACT)
      throw hdf5_exception("lod::store: dataset '" + name +
                           "' is not contiguous or compact (chunked or virtual layout)");
    return ds;
  }

  // Refuses an object this call has already reached under another name.
  void reached(const H5Object &obj, const std::string &what) {
    if (!_seen.insert(object_address(obj.getId())).second)
      throw hdf5_exception("lod::store: '" + what +
                           "' is an object this call already read, linked under a second name; "
                           "a scene.cvch5 links each object once");
  }

  // Charges `stored` bytes read from the container, decoded into `decoded`.
  void charge(hsize_t stored, hsize_t decoded, const std::string &what) {
    const hsize_t decode_limit = mul_sat(_bytes, 8);
    if (stored > _bytes - _stored || decoded > decode_limit - _decoded)
      throw hdf5_exception("lod::store: " + what +
                           " takes this call past the bytes the container holds (" +
                           std::to_string(_bytes) + "); some data is read more than once");
    _stored += stored;
    _decoded += decoded;
  }

private:
  H5File &_f;
  access_plist _lapl, _gapl, _dapl;
  hsize_t _bytes;
  hsize_t _stored = 0, _decoded = 0;
  std::set<haddr_t> _seen;
};

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

// A reader sizes its buffer from a dataset's extent before HDF5 reads a byte,
// and that extent is untrusted: a small crafted blob can declare a huge one over
// storage it never allocated (read back as fill values), or storage running past
// its own end. The store writes every dataset contiguous and unfiltered, so its
// `rows` x `per_row` elements are all physically in the file: refuse one whose
// storage holds fewer, or more than the file itself (`file_bytes`), before
// allocating anything. (Were the format ever to compress datasets, this bound
// would have to come from somewhere else.)
void check_stored(const DataSet &ds, hsize_t rows, hsize_t per_row, hsize_t file_bytes,
                  const std::string &what) {
  if (rows == 0 || per_row == 0)
    return;
  const hsize_t esz = ds.getDataType().getSize();
  const hsize_t stored = ds.getStorageSize();
  if (esz == 0 || stored > file_bytes || rows > stored / esz / per_row) // no overflow
    throw hdf5_exception("lod::store: " + what + " declares more data than the file stores");
}

// check_stored(), then charges the dataset to the call's budget: its stored
// bytes, and the rows x per_row elements of `mem_size` bytes it decodes into.
void charge_dataset(reader_call &c, const DataSet &ds, hsize_t rows, hsize_t per_row,
                    std::size_t mem_size, const std::string &what) {
  check_stored(ds, rows, per_row, c.container_bytes(), what);
  c.charge(ds.getStorageSize(), mul_sat(mul_sat(rows, per_row), mem_size), what);
}

template <class T>
hsize_t read_2d(reader_call &c, const Group &g, const char *name, hsize_t cols, std::vector<T> &out,
                const PredType &pt) {
  out.clear();
  if (!c.has(g, name))
    return 0;
  DataSet ds = c.dataset(g, name);
  DataSpace sp = ds.getSpace();
  if (sp.getSimpleExtentNdims() != 2)
    throw hdf5_exception(std::string("lod::store: rank!=2 for ") + name);
  hsize_t dims[2] = {0, 0};
  sp.getSimpleExtentDims(dims);
  if (dims[1] != cols)
    throw hdf5_exception(std::string("lod::store: cols mismatch for ") + name);
  charge_dataset(c, ds, dims[0], cols, sizeof(T), name);
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

// The asset group `name` of `kind` and its rung groups lod/0 .. lod/nrungs-1,
// each reached once. nrungs is only an attribute: the lod group must hold at
// least that many links (every rung then costs the container a group).
Group open_asset(reader_call &c, char kind, const std::string &name, const char *call,
                 std::vector<Group> &rungs) {
  Group ag;
  if (!c.find_group(asset_path(kind, name), ag))
    throw hdf5_exception(std::string(call) + ": no " + (kind == 'I' ? "image" : "mesh") +
                         " pyramid '" + name + "'");
  c.reached(ag, name);
  int nrungs = 0;
  hu::getAttribute<int>(ag, "nrungs", nrungs);
  rungs.clear();
  if (nrungs <= 0)
    return ag;
  const Group lod = c.group(ag, "lod");
  const hsize_t links = lod.getNumObjs();
  if (hsize_t(nrungs) > links)
    throw hdf5_exception(std::string(call) + ": '" + name + "' claims " + std::to_string(nrungs) +
                         " rungs but its lod group holds " + std::to_string(links));
  rungs.reserve(std::size_t(nrungs));
  for (int k = 0; k < nrungs; ++k)
    rungs.push_back(c.group(lod, std::to_string(k)));
  return ag;
}

mesh_pyramid read_mesh_from(app &ctx, reader_call &c, const std::string &name) {
  mesh_pyramid out;
  std::vector<Group> rungs;
  open_asset(c, 'M', name, "lod::read_mesh_pyramid", rungs);
  for (const Group &rg : rungs) {
    geometry m(ctx);
    std::vector<double> P;
    hsize_t nv = read_2d(c, rg, "points", 3, P, PredType::NATIVE_DOUBLE);
    std::vector<std::uint64_t> Tt;
    hsize_t nt = read_2d(c, rg, "tris", 3, Tt, PredType::NATIVE_UINT64);
    // A triangle naming a vertex past the end would send every consumer of
    // the mesh reading out of bounds.
    for (std::uint64_t v : Tt)
      if (v >= nv)
        throw hdf5_exception("lod::read_mesh_pyramid: triangle index out of range in '" + name +
                             "'");
    geometry::points_t &mp = m.points();
    mp.resize(nv);
    for (hsize_t i = 0; i < nv; ++i)
      mp[i] = {P[i * 3], P[i * 3 + 1], P[i * 3 + 2]};
    geometry::tris_t &mt = m.tris();
    mt.resize(nt);
    for (hsize_t i = 0; i < nt; ++i)
      mt[i] = {Tt[i * 3], Tt[i * 3 + 1], Tt[i * 3 + 2]};
    std::vector<double> U;
    hsize_t nu = read_2d(c, rg, "uv", 2, U, PredType::NATIVE_DOUBLE);
    if (nu == nv && nv) {
      geometry::uvs_t &mu = m.uvs();
      mu.resize(nv);
      for (hsize_t i = 0; i < nv; ++i)
        mu[i] = {U[i * 2], U[i * 2 + 1]};
    }
    std::vector<double> Cc;
    hsize_t nc = read_2d(c, rg, "colors", 3, Cc, PredType::NATIVE_DOUBLE);
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

image_pyramid read_image_from(reader_call &c, const std::string &name) {
  image_pyramid out;
  std::vector<Group> rungs;
  open_asset(c, 'I', name, "lod::read_image_pyramid", rungs);
  for (const Group &rg : rungs) {
    int w = 0, h = 0, fmt = int(image::pixel_format::RGBA);
    hu::getAttribute<int>(rg, "w", w);
    hu::getAttribute<int>(rg, "h", h);
    hu::getAttribute<int>(rg, "format", fmt);
    if (fmt < int(image::pixel_format::GRAY) || fmt > int(image::pixel_format::RGBA))
      throw hdf5_exception("lod::read_image_pyramid: bad pixel format in '" + name + "'");
    // A negative size would make an image whose size_bytes() wraps to ~2^64.
    if (w < 0 || h < 0)
      throw hdf5_exception("lod::read_image_pyramid: negative size in '" + name + "'");
    const image::pixel_format pf = image::pixel_format(fmt);
    const int nc = image(0, 0, pf).channels(); // allocates nothing
    // The read fills the whole dataset into the image's buffer, so the dataset
    // must be exactly the H x W x C the attributes size it for, and actually
    // stored -- all checked BEFORE the image allocates (and zero-fills) that
    // buffer, since a blob is untrusted input.
    DataSet ds = c.dataset(rg, "pixels");
    DataSpace sp = ds.getSpace();
    hsize_t dims[3] = {0, 0, 0};
    const bool rank3 = sp.getSimpleExtentNdims() == 3;
    if (rank3)
      sp.getSimpleExtentDims(dims);
    if (!rank3 || dims[0] != hsize_t(h) || dims[1] != hsize_t(w) || dims[2] != hsize_t(nc))
      throw hdf5_exception("lod::read_image_pyramid: pixels do not match w/h/format in '" + name +
                           "'");
    charge_dataset(c, ds, dims[0], dims[1] * dims[2], 1, "pixels of '" + name + "'");
    image im(w, h, pf, image::data_type::u8);
    if (im.size_bytes())
      ds.read(im.data(), PredType::NATIVE_UINT8);
    double we = 0.0;
    hu::getAttribute<double>(rg, "world_error_m", we);
    out.rungs.push_back(std::move(im));
    out.world_error_m.push_back(we);
  }
  return out;
}

std::vector<lod_index_entry> read_index_from(reader_call &c) {
  std::vector<lod_index_entry> out;
  Group cvc;
  if (!c.find_group("/cvc", cvc))
    return out;
  const char kinds[3] = {'M', 'I', 'V'};
  for (char kind : kinds) {
    if (!c.has(cvc, kind_group(kind)))
      continue;
    const Group g = c.group(cvc, kind_group(kind));
    const hsize_t n = g.getNumObjs();
    for (hsize_t i = 0; i < n; ++i) {
      std::string name = g.getObjnameByIdx(i);
      const Group ag = c.group(g, name);
      lod_index_entry e;
      e.name = name;
      e.kind = kind;
      hu::getAttribute<int>(ag, "nrungs", e.nrungs);
      try {
        hu::getAttribute<std::string>(ag, "source_hash", e.source_hash);
      } catch (...) {
      }
      if (ag.attrExists("world_error_m") && e.nrungs > 0) {
        // Check the stored length BEFORE sizing a buffer by nrungs: a corrupt
        // nrungs (up to INT_MAX) would otherwise zero-fill gigabytes first.
        const Attribute we = ag.openAttribute("world_error_m");
        const hssize_t n = we.getSpace().getSimpleExtentNpoints();
        if (n != hssize_t(e.nrungs))
          throw hdf5_exception("lod::read_lod_index: '" + name + "' has " + std::to_string(n) +
                               " world_error_m entries for nrungs " + std::to_string(e.nrungs));
        c.charge(we.getStorageSize(), mul_sat(hsize_t(n), sizeof(double)),
                 "world_error_m of '" + name + "'");
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

hdf5_exception h5_failure(const char *call, const std::string &file, const H5::Exception &e,
                          const char *hint = nullptr) {
  // The wrapper's text ("H5Fopen failed") says little; the innermost entry of
  // HDF5's error stack ("file signature not found") says why. Any later HDF5
  // call -- a handle closed while unwinding -- clears that stack, so this is
  // best-effort unless called right at the failure.
  std::string cause;
  H5Ewalk2(H5E_DEFAULT, H5E_WALK_UPWARD, innermost_error, &cause);
  std::string msg = std::string(call) + " '" + file + "': ";
  if (hint)
    msg += std::string(hint) + ": ";
  msg += e.getFuncName() + ": " + e.getDetailMsg();
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
// unique across threads; a random per-process nonce keeps two statically linked
// copies of libcvc that share one HDF5 apart (each has a counter of its own).
// The name appears in error messages, which may be logged or sent anywhere, so
// it carries nothing about the process -- no address. Nothing is created on
// disk (backing_store is off) -- the name is only an identity.
std::uint64_t process_nonce() {
  try {
    std::random_device rd;
    return (std::uint64_t(rd()) << 32) ^ std::uint64_t(rd());
  } catch (...) { // no entropy source: the clock still tells two copies apart
    return std::uint64_t(std::chrono::steady_clock::now().time_since_epoch().count());
  }
}
std::string unique_core_name(const char *what) {
  static const std::uint64_t nonce = process_nonce();
  static std::atomic<std::uint64_t> counter(0);
  std::ostringstream os;
  os << "cvc-lod-" << what << '-' << std::hex << nonce << std::dec << '-'
     << counter.fetch_add(1, std::memory_order_relaxed) << ".cvch5";
  return os.str();
}

// Trust (inc/cvc/lod/store.h; roadmap D11): a blob handed to the plain constructor
// is from a TRUSTED source, as a local file is, and parses with any HDF5; bytes
// from a source the application does not control go through open_verified. No
// HDF5 release is memory-safe on a crafted file (e.g. CVE-2025-6516 through
// 1.14.6), which is why authentication is offered at all; the structural hardening
// below applies to every container whichever way it is opened.

bool is_sha256_hex(const std::string &s) {
  return s.size() == 64 && std::all_of(s.begin(), s.end(), [](char ch) {
           return std::isxdigit(static_cast<unsigned char>(ch)) != 0;
         });
}
std::string lower(std::string s) {
  for (char &ch : s)
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  return s;
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
boost::shared_ptr<H5File> open_blob_file(const std::string &name, const unsigned char *bytes,
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
// A writer opens an existing scene read-write, preserving its assets, and
// creates one only where nothing exists yet. It never truncates: an existing
// path it cannot open read-write -- not HDF5 (a mistyped path), damaged (cut
// short), read-only, or held open by a live scene_reader -- is an error, not
// something to replace with an empty scene. (hdf5_utils::getH5File would
// truncate the first two and quietly fall back to read-only for the others.)
boost::shared_ptr<H5File> open_writable(const std::string &path) {
  try {
    return boost::make_shared<H5File>(path, H5F_ACC_RDWR);
  } catch (const H5::Exception &e) {
    std::error_code ec;
    if (std::filesystem::exists(path, ec) || ec) // unsure counts as present
      throw h5_failure("lod::scene_writer", path, e,
                       "will not overwrite a path it cannot open read-write (not an HDF5 scene, "
                       "damaged, read-only, or a scene_reader has it open)");
  }
  // Nothing there: create it. EXCL rather than TRUNC, so a file that appears in
  // the meantime makes this fail instead of being clobbered.
  return boost::make_shared<H5File>(path, H5F_ACC_EXCL);
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
  std::size_t blob_bytes; // 0 for a file
  impl(app &c, boost::shared_ptr<H5File> ff, std::string k, std::size_t n = 0)
      : ctx(c), f(ff), key(std::move(k)), blob_bytes(n) {}
  // What a call's byte budget is measured against: exactly the bytes a blob
  // came as; a file's size as HDF5 sees it, which grows as a live writer on
  // the same path adds to it.
  hsize_t container_bytes() const { return blob_bytes ? hsize_t(blob_bytes) : f->getFileSize(); }
};

scene_reader::scene_reader() = default;
scene_reader::scene_reader(app &ctx, const std::string &path) {
  hu::library_lock lock(ctx, path, "cvc::lod::scene_reader(file)");
  H5::Exception::dontPrint();
  _p.reset(
      new impl(ctx, guarded("lod::scene_reader", path, [&] { return open_existing(path); }), path));
}
scene_reader::scene_reader(app &ctx, const unsigned char *bytes, std::size_t n) {
  open_bytes(ctx, bytes, n);
}
scene_reader scene_reader::open_verified(app &ctx, const unsigned char *bytes, std::size_t n,
                                         const std::string &expected_sha256_hex) {
  // Before any HDF5 call -- nothing unauthenticated reaches the parser -- and
  // without the library lock, so hashing a large blob stalls no other HDF5 user.
  if (!is_sha256_hex(expected_sha256_hex))
    throw std::runtime_error("lod::scene_reader::open_verified: the expected hash is not a "
                             "SHA-256 digest (64 hex digits)");
  // The caller's buffer is read ONCE, into a private copy that is both hashed
  // and parsed. Hashing the caller's bytes and handing HDF5 the caller's bytes
  // would read them twice, with the wait for the library lock in between: a
  // buffer that changes meanwhile (mmapped, shared, reused for the next
  // download) would put bytes nobody authenticated into the parser.
  const std::vector<unsigned char> own(bytes, bytes ? bytes + n : bytes);
  const std::string actual = sha256_hex(own);
  if (actual != lower(expected_sha256_hex))
    throw std::runtime_error("lod::scene_reader::open_verified: the blob's SHA-256 is " + actual +
                             ", not the expected " + lower(expected_sha256_hex) +
                             "; refusing to parse it");
  scene_reader r;
  r.open_bytes(ctx, own.data(), own.size());
  return r;
}
void scene_reader::open_bytes(app &ctx, const unsigned char *bytes, std::size_t n) {
  const std::string name = unique_core_name("blob");
  hu::library_lock lock(ctx, name, "cvc::lod::scene_reader(blob)");
  H5::Exception::dontPrint();
  _p.reset(new impl(
      ctx, guarded("lod::scene_reader", name, [&] { return open_blob_file(name, bytes, n); }), name,
      n));
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

// Each public read is one reader_call: one byte budget, one set of reached
// objects (store.h, trust model).
mesh_pyramid scene_reader::read_mesh_pyramid(const std::string &name) {
  hu::library_lock lock(_p->ctx, _p->key, "cvc::lod::scene_reader::read_mesh_pyramid");
  H5::Exception::dontPrint();
  return guarded("lod::scene_reader::read_mesh_pyramid", _p->key, [&] {
    reader_call c(*_p->f, _p->container_bytes());
    return read_mesh_from(_p->ctx, c, name);
  });
}
image_pyramid scene_reader::read_image_pyramid(const std::string &name) {
  hu::library_lock lock(_p->ctx, _p->key, "cvc::lod::scene_reader::read_image_pyramid");
  H5::Exception::dontPrint();
  return guarded("lod::scene_reader::read_image_pyramid", _p->key, [&] {
    reader_call c(*_p->f, _p->container_bytes());
    return read_image_from(c, name);
  });
}
std::vector<lod_index_entry> scene_reader::index() {
  hu::library_lock lock(_p->ctx, _p->key, "cvc::lod::scene_reader::index");
  H5::Exception::dontPrint();
  return guarded("lod::scene_reader::index", _p->key, [&] {
    reader_call c(*_p->f, _p->container_bytes());
    return read_index_from(c);
  });
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
