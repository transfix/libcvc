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

// cvc::class_surfaces. See inc/cvc/volume/class_surfaces.h.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cvc/volren/detail/mc_tables.h>
#include <cvc/volume/class_surfaces.h>
#include <cvc/volume/volume_slice.h>
#include <exception>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace cvc {

namespace {

namespace mc = volren::detail;

// The corners of a cell in the order the marching-cubes tables number them (mc_tables.h: 0-3 ring
// the y = 0 face, 4-7 the y = 1 face), as offsets along x, y and z.
constexpr int kCorner[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1},
                               {0, 1, 0}, {1, 1, 0}, {1, 1, 1}, {0, 1, 1}};

// Units between two looks at cancel() inside one step.
constexpr std::size_t kCancelEvery = 1024;

std::string fmt(const char *f, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}

using point_t = geometry::point_t;
using vector_t = geometry::vector_t;

vector_t cross(const point_t &a, const point_t &b, const point_t &c) {
  const double u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
  const double w[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
  return {{u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]}};
}

} // namespace

const geometry *class_surface_result::surface_of(int label) const {
  const auto it = std::lower_bound(labels.begin(), labels.end(), label);
  if (it == labels.end() || *it != label)
    return nullptr;
  return &surfaces[static_cast<std::size_t>(it - labels.begin())];
}

std::size_t class_surface_result::triangle_count() const {
  std::size_t n = 0;
  for (const geometry &g : surfaces)
    n += g.tris().size();
  return n;
}

struct class_surfaces_stepper::impl {
  enum class stage { checking, reading, indexing, extracting, adjacency, smoothing, normals, done };

  volume in;
  class_surface_params p;
  std::atomic<bool> cancel{false};
  bool failed = false;
  std::exception_ptr error;
  double prog = 0.0;
  stage at = stage::checking;

  // The grid.
  std::int64_t X = 0, Y = 0, Z = 0;
  std::int64_t dims[3] = {0, 0, 0};
  std::size_t n = 0;
  double lo[3] = {0, 0, 0}, sp[3] = {1, 1, 1};

  // reading and indexing: each voxel's label, then its class slot (-1: none).
  std::vector<std::int32_t> lab;
  std::size_t v = 0;
  std::unordered_set<std::int32_t> seen;
  std::int32_t last_seen = 0;
  bool have_last = false;
  std::vector<int> wanted;         // the labels surfaced, ascending (the slots)
  std::vector<std::int32_t> dense; // label - dense_lo -> slot, when the labels span little
  std::int32_t dense_lo = 0;
  std::vector<std::size_t> voxels; // per slot

  // extracting
  std::int64_t c0 = 0, cx = 0, cy = 0, cz = 0;
  std::size_t cell = 0, cells = 0;
  std::vector<geometry> mesh;
  std::vector<std::unordered_map<std::uint64_t, std::uint32_t>> vmap;

  // smoothing and normals: the slot being worked on, and where in it.
  std::size_t s = 0, pass = 0, k = 0;
  std::vector<std::vector<std::uint32_t>> nbr;
  geometry::points_t moved;
  std::size_t post_units = 0, post_total = 0;    // smoothing's
  std::size_t normal_units = 0, normal_total = 0; // normals'

  std::vector<std::string> warnings;
  std::optional<class_surface_result> result;

  impl(const volume &vol, class_surface_params params) : in(vol), p(std::move(params)) {}

  const char *phase() const {
    if (failed)
      return "failed";
    switch (at) {
    case stage::checking:
      return "checking";
    case stage::reading:
    case stage::indexing:
      return "reading";
    case stage::extracting:
      return "extracting";
    case stage::adjacency:
    case stage::smoothing:
      return "smoothing";
    case stage::normals:
      return "normals";
    case stage::done:
      return "done";
    }
    return "done";
  }

  void note_progress() {
    double lo_ = 0, hi_ = 0, within = 0;
    switch (at) {
    case stage::checking:
      break;
    case stage::reading:
      lo_ = 0.0, hi_ = 0.15, within = 0.5 * double(v) / double(n ? n : 1);
      break;
    case stage::indexing:
      lo_ = 0.0, hi_ = 0.15, within = 0.5 + 0.5 * double(v) / double(n ? n : 1);
      break;
    case stage::extracting:
      lo_ = 0.15, hi_ = 0.6, within = double(cell) / double(cells ? cells : 1);
      break;
    case stage::adjacency:
    case stage::smoothing:
      lo_ = 0.6, hi_ = 0.95, within = double(post_units) / double(post_total ? post_total : 1);
      break;
    case stage::normals:
      lo_ = 0.95, hi_ = 1.0,
      within = double(normal_units) / double(normal_total ? normal_total : 1);
      break;
    case stage::done:
      lo_ = hi_ = within = 1.0;
      break;
    }
    const double q = lo_ + (hi_ - lo_) * std::min(1.0, std::max(0.0, within));
    prog = std::max(prog, std::min(q, 1.0));
  }

  // One unit of work; false once there is no more.
  void check();
  bool read_one();
  void begin_indexing();
  bool index_one();
  void begin_extracting();
  bool extract_one();
  void begin_post();
  bool adjacency_one();
  bool smooth_one();
  bool normal_one();
  void finish();

  std::int32_t slot_of(std::int32_t label) const {
    if (!dense.empty()) {
      const std::int64_t d = std::int64_t(label) - dense_lo;
      return d >= 0 && d < std::int64_t(dense.size()) ? dense[std::size_t(d)] : -1;
    }
    const auto it = std::lower_bound(wanted.begin(), wanted.end(), label);
    return it != wanted.end() && *it == label ? std::int32_t(it - wanted.begin()) : -1;
  }
  std::int32_t corner(std::int64_t i, std::int64_t j, std::int64_t kk) const {
    if (i < 0 || j < 0 || kk < 0 || i >= X || j >= Y || kk >= Z)
      return -1;
    return lab[std::size_t((kk * Y + j) * X + i)];
  }
};

void class_surfaces_stepper::impl::check() {
  const std::vector<std::string> problems = params_validate(p);
  if (!problems.empty()) {
    std::string m = "class_surfaces: ";
    for (std::size_t i = 0; i < problems.size(); ++i)
      m += (i ? "; " : "") + problems[i];
    throw class_surfaces_error(m);
  }
  if (in.voxelType() == Undefined)
    throw class_surfaces_error("class_surfaces: the volume's voxel type is undefined");
  X = std::int64_t(in.XDim());
  Y = std::int64_t(in.YDim());
  Z = std::int64_t(in.ZDim());
  n = std::size_t(X) * std::size_t(Y) * std::size_t(Z);
  dims[0] = X;
  dims[1] = Y;
  dims[2] = Z;
  const slice_grid g = grid_of(in);
  for (int a = 0; a < 3; ++a) {
    lo[a] = g.lo(a);
    sp[a] = g.spacing(a);
  }
  lab.assign(n, 0);
  v = 0;
  at = stage::reading;
}

bool class_surfaces_stepper::impl::read_one() {
  if (v == n) {
    begin_indexing();
    return true;
  }
  const volume &cv = in; // read through const: a non-const access would copy the voxels
  std::int32_t l;
  if (cv.voxelType() == UChar) {
    l = cv.data_ptr()[v];
  } else {
    const double d = cv(static_cast<uint64>(v));
    if (!std::isfinite(d) || d != std::floor(d) ||
        d < double(std::numeric_limits<std::int32_t>::min()) ||
        d > double(std::numeric_limits<std::int32_t>::max())) {
      const std::size_t i = v % std::size_t(X), j = (v / std::size_t(X)) % std::size_t(Y),
                        kk = v / (std::size_t(X) * std::size_t(Y));
      throw class_surfaces_error(fmt("class_surfaces: voxel (%zu, %zu, %zu) holds %g, which is "
                                     "not a label",
                                     i, j, kk, d));
    }
    l = std::int32_t(d);
  }
  lab[v++] = l;
  if (!have_last || l != last_seen) {
    seen.insert(l);
    last_seen = l;
    have_last = true;
  }
  return true;
}

void class_surfaces_stepper::impl::begin_indexing() {
  std::vector<std::int32_t> all(seen.begin(), seen.end());
  std::sort(all.begin(), all.end());
  seen.clear();
  if (p.labels.empty()) {
    for (std::int32_t l : all)
      if (l != p.background)
        wanted.push_back(l);
  } else {
    std::vector<int> ask = p.labels;
    std::sort(ask.begin(), ask.end());
    ask.erase(std::unique(ask.begin(), ask.end()), ask.end());
    for (int l : ask) {
      if (std::binary_search(all.begin(), all.end(), l))
        wanted.push_back(l);
      else
        warnings.push_back(fmt("label %d is not in the volume, so it has no surface", l));
    }
  }
  if (!wanted.empty() && std::int64_t(wanted.back()) - wanted.front() < (std::int64_t(1) << 20)) {
    dense_lo = wanted.front();
    dense.assign(std::size_t(wanted.back() - wanted.front() + 1), -1);
    for (std::size_t i = 0; i < wanted.size(); ++i)
      dense[std::size_t(wanted[i] - dense_lo)] = std::int32_t(i);
  }
  voxels.assign(wanted.size(), 0);
  v = 0;
  at = stage::indexing;
}

bool class_surfaces_stepper::impl::index_one() {
  if (v == n) {
    begin_extracting();
    return true;
  }
  const std::int32_t s_ = slot_of(lab[v]);
  lab[v++] = s_;
  if (s_ >= 0)
    ++voxels[std::size_t(s_)];
  return true;
}

void class_surfaces_stepper::impl::begin_extracting() {
  c0 = p.close ? -1 : 0;
  cx = p.close ? X + 1 : X - 1;
  cy = p.close ? Y + 1 : Y - 1;
  cz = p.close ? Z + 1 : Z - 1;
  cells = (cx > 0 && cy > 0 && cz > 0 && !wanted.empty())
              ? std::size_t(cx) * std::size_t(cy) * std::size_t(cz)
              : 0;
  cell = 0;
  mesh.assign(wanted.size(), geometry());
  vmap.assign(wanted.size(), {});
  at = stage::extracting;
}

bool class_surfaces_stepper::impl::extract_one() {
  if (cell == cells) {
    vmap.clear();
    vmap.shrink_to_fit();
    lab.clear();
    lab.shrink_to_fit();
    begin_post();
    return true;
  }
  const std::int64_t ci = c0 + std::int64_t(cell % std::size_t(cx));
  const std::int64_t cj = c0 + std::int64_t((cell / std::size_t(cx)) % std::size_t(cy));
  const std::int64_t ck = c0 + std::int64_t(cell / (std::size_t(cx) * std::size_t(cy)));
  ++cell;
  std::int32_t c[8];
  bool same = true;
  for (int q = 0; q < 8; ++q) {
    c[q] = corner(ci + kCorner[q][0], cj + kCorner[q][1], ck + kCorner[q][2]);
    same = same && c[q] == c[0];
  }
  if (same)
    return true;
  // Each class among the corners, once: its mask's case, and the triangles of that case.
  for (int q = 0; q < 8; ++q) {
    const std::int32_t L = c[q];
    if (L < 0)
      continue;
    bool first = true;
    for (int r = 0; r < q; ++r)
      first = first && c[r] != L;
    if (!first)
      continue;
    int code = 0;
    for (int r = 0; r < 8; ++r)
      if (c[r] == L)
        code |= 1 << r;
    geometry &g = mesh[std::size_t(L)];
    auto &map = vmap[std::size_t(L)];
    const signed char *tri = mc::tri_cases[code];
    for (int t = 0; t < 16 && tri[t] >= 0; t += 3) {
      std::uint32_t idx[3];
      for (int e = 0; e < 3; ++e) {
        const mc::mc_edge_info &ed = mc::mc_edges[tri[t + e]];
        // The edge's start corner, in the grid padded by one voxel each side, and its axis.
        const std::int64_t si = ci + ed.di, sj = cj + ed.dj, sk = ck + ed.dk;
        const std::uint64_t key =
            ((std::uint64_t(sk + 1) * std::uint64_t(Y + 2) + std::uint64_t(sj + 1)) *
                 std::uint64_t(X + 2) +
             std::uint64_t(si + 1)) *
                3 +
            std::uint64_t(ed.dir);
        const auto it = map.find(key);
        if (it != map.end()) {
          idx[e] = it->second;
        } else {
          double at_[3] = {double(si), double(sj), double(sk)};
          at_[ed.dir] += 0.5;
          const std::uint32_t id = std::uint32_t(g.points().size());
          g.points().push_back(
              {{lo[0] + at_[0] * sp[0], lo[1] + at_[1] * sp[1], lo[2] + at_[2] * sp[2]}});
          map.emplace(key, id);
          idx[e] = id;
        }
      }
      // The tables wind for x, z, y corners (mc_tables.h numbers them y and z swapped from VTK's,
      // whose cases they are), so each triangle is turned over to face out of its class.
      g.tris().push_back({{idx[0], idx[2], idx[1]}});
    }
  }
  return true;
}

void class_surfaces_stepper::impl::begin_post() {
  post_total = 0;
  normal_total = 0;
  for (std::size_t q = 0; q < mesh.size(); ++q) {
    if (voxels[q] > 0 && mesh[q].tris().empty())
      warnings.push_back(fmt("label %d fills the volume, so it has no surface (close is off)",
                             wanted[q]));
    if (p.smoothing > 0)
      post_total += mesh[q].tris().size() + 2 * std::size_t(p.smoothing) * mesh[q].points().size();
    normal_total += mesh[q].tris().size() + mesh[q].points().size();
  }
  s = pass = k = 0;
  post_units = normal_units = 0;
  at = p.smoothing > 0 ? stage::adjacency : stage::normals;
}

// Each vertex's neighbours: the vertices it shares a triangle with, once each.
bool class_surfaces_stepper::impl::adjacency_one() {
  if (s == mesh.size()) {
    s = k = 0;
    at = stage::normals;
    return true;
  }
  geometry &g = mesh[s];
  if (k == 0 && nbr.size() != g.points().size())
    nbr.assign(g.points().size(), {});
  if (k == g.tris().size()) {
    // This class's neighbours are known: its passes next.
    pass = 0;
    k = 0;
    moved = g.points();
    at = stage::smoothing;
    return true;
  }
  const geometry::tri_t &t = g.tris()[k++];
  for (int a = 0; a < 3; ++a)
    for (int b = 0; b < 3; ++b) {
      if (a == b)
        continue;
      std::vector<std::uint32_t> &l = nbr[std::size_t(t[a])];
      const std::uint32_t w = std::uint32_t(t[b]);
      if (std::find(l.begin(), l.end(), w) == l.end())
        l.push_back(w);
    }
  ++post_units;
  return true;
}

bool class_surfaces_stepper::impl::smooth_one() {
  geometry &g = mesh[s];
  geometry::points_t &P = g.points();
  if (k == P.size()) {
    P.swap(moved); // the pass's positions
    k = 0;
    if (++pass == 2 * std::size_t(p.smoothing)) {
      // The next class.
      nbr.clear();
      moved.clear();
      ++s;
      at = stage::adjacency;
    }
    return true;
  }
  const double f = pass % 2 == 0 ? p.lambda : p.mu;
  const std::vector<std::uint32_t> &l = nbr[k];
  point_t q = P[k];
  if (!l.empty()) {
    double m[3] = {0, 0, 0};
    for (std::uint32_t w : l)
      for (int a = 0; a < 3; ++a)
        m[a] += P[w][a];
    for (int a = 0; a < 3; ++a) {
      q[a] += f * (m[a] / double(l.size()) - q[a]);
      // Kept within the box half a voxel past the outer voxels (where the closing caps lie):
      // a mu step can push a cap's rim a little past it.
      const double lim[2] = {lo[a] - 0.5 * sp[a], lo[a] + (double(dims[a]) - 0.5) * sp[a]};
      q[a] = std::min(std::max(q[a], std::min(lim[0], lim[1])), std::max(lim[0], lim[1]));
    }
  }
  moved[k++] = q;
  ++post_units;
  return true;
}

bool class_surfaces_stepper::impl::normal_one() {
  if (s == mesh.size()) {
    finish();
    return false;
  }
  geometry &g = mesh[s];
  const std::size_t T = g.tris().size(), V = g.points().size();
  if (k == 0 && g.normals().size() != V)
    g.normals().assign(V, vector_t{{0.0, 0.0, 0.0}});
  if (k < T) {
    // The triangle's area-weighted normal, added to its corners'.
    const geometry::tri_t &t = g.tris()[k];
    const vector_t c = cross(g.points()[t[0]], g.points()[t[1]], g.points()[t[2]]);
    for (int a = 0; a < 3; ++a)
      for (int d = 0; d < 3; ++d)
        g.normals()[std::size_t(t[a])][d] += c[d];
  } else if (k < T + V) {
    vector_t &nn = g.normals()[k - T];
    const double len = std::sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]);
    if (len > 0)
      for (int d = 0; d < 3; ++d)
        nn[d] /= len;
  } else {
    ++s;
    k = 0;
    return true;
  }
  ++k;
  ++normal_units;
  return true;
}

void class_surfaces_stepper::impl::finish() {
  class_surface_result &r = result.emplace();
  for (std::size_t q = 0; q < mesh.size(); ++q) {
    if (mesh[q].tris().empty())
      continue;
    r.labels.push_back(wanted[q]);
    r.surfaces.push_back(std::move(mesh[q]));
    r.voxels.push_back(voxels[q]);
  }
  mesh.clear();
  r.warnings = std::move(warnings);
  at = stage::done;
  prog = 1.0;
}

class_surfaces_stepper::class_surfaces_stepper(const volume &labels, class_surface_params params)
    : _impl(std::make_unique<impl>(labels, std::move(params))) {}

class_surfaces_stepper::~class_surfaces_stepper() = default;
class_surfaces_stepper::class_surfaces_stepper(class_surfaces_stepper &&) noexcept = default;
class_surfaces_stepper &
class_surfaces_stepper::operator=(class_surfaces_stepper &&) noexcept = default;

bool class_surfaces_stepper::step(std::size_t budget) {
  if (!_impl)
    return true;
  impl &m = *_impl;
  if (m.failed) {
    if (m.cancel.load(std::memory_order_acquire))
      return true;
    std::rethrow_exception(m.error);
  }
  if (m.at == impl::stage::done)
    return true;
  try {
    std::size_t left = budget == 0 ? std::numeric_limits<std::size_t>::max() : budget;
    std::size_t since = 0;
    while (left > 0 && m.at != impl::stage::done) {
      if (++since == kCancelEvery) {
        since = 0;
        if (m.cancel.load(std::memory_order_acquire))
          break;
      }
      switch (m.at) {
      case impl::stage::checking:
        m.check();
        break;
      case impl::stage::reading:
        m.read_one();
        break;
      case impl::stage::indexing:
        m.index_one();
        break;
      case impl::stage::extracting:
        m.extract_one();
        break;
      case impl::stage::adjacency:
        m.adjacency_one();
        break;
      case impl::stage::smoothing:
        m.smooth_one();
        break;
      case impl::stage::normals:
        m.normal_one();
        break;
      case impl::stage::done:
        break;
      }
      if (left != std::numeric_limits<std::size_t>::max())
        --left;
    }
    if (m.cancel.load(std::memory_order_acquire) && m.at != impl::stage::done) {
      m.error = std::make_exception_ptr(class_surfaces_error("class_surfaces: cancelled"));
      m.failed = true;
      m.lab.clear();
      m.mesh.clear();
      m.vmap.clear();
      m.nbr.clear();
      m.result.reset();
      return true;
    }
    m.note_progress();
    return m.at == impl::stage::done;
  } catch (...) {
    m.error = std::current_exception();
    m.failed = true;
    throw;
  }
}

bool class_surfaces_stepper::done() const {
  return !_impl || _impl->at == impl::stage::done || _impl->failed;
}
const char *class_surfaces_stepper::phase() const { return _impl ? _impl->phase() : "done"; }
double class_surfaces_stepper::progress() const { return _impl ? _impl->prog : 1.0; }

std::string class_surfaces_stepper::status() const {
  if (!_impl)
    return "done";
  const impl &m = *_impl;
  if (m.failed) {
    try {
      std::rethrow_exception(m.error);
    } catch (const std::exception &e) {
      return std::string("failed: ") + e.what();
    } catch (...) {
      return "failed";
    }
  }
  if (m.at == impl::stage::done && m.result)
    return fmt("done: %zu surfaces, %zu triangles", m.result->labels.size(),
               m.result->triangle_count());
  return fmt("%s %d%%", m.phase(), int(m.prog * 100));
}

void class_surfaces_stepper::cancel() {
  if (_impl)
    _impl->cancel.store(true, std::memory_order_release);
}

class_surface_result class_surfaces_stepper::take() {
  if (_impl && _impl->failed)
    std::rethrow_exception(_impl->error);
  if (!_impl || _impl->at != impl::stage::done || !_impl->result)
    throw class_surfaces_error("class_surfaces: take() before the stepper is done");
  class_surface_result r = std::move(*_impl->result);
  _impl.reset();
  return r;
}

class_surface_result class_surfaces(const volume &labels, const class_surface_params &params) {
  class_surfaces_stepper s(labels, params);
  s.step(0);
  return s.take();
}

std::unique_ptr<result_job<class_surface_result>>
class_surfaces_job(const volume &labels, class_surface_params params) {
  return make_result_job(std::make_unique<class_surfaces_stepper>(labels, std::move(params)));
}

} // namespace cvc
