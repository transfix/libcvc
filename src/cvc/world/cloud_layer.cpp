/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

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

// cloud_layer.cpp — the world-scale cloud deck (see cloud_layer.h).

#include <algorithm>
#include <cmath>
#include <cvc/lsys/cloud.h>
#include <cvc/lsys/rng.h>
#include <cvc/world/cloud_layer.h>
#include <numeric>

namespace cvc {
namespace world {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double LUMP_FLOOR = 0.10;    // the density below which a map is clear sky
constexpr double TAPER_FROM = 0.6;     // a map fades out from this fraction of its half-width
constexpr int TAU_STEPS = 32;          // samples along the sun through the deck, per shadow texel
constexpr int FAR_SUB = 3;             // sub-samples per far texel and axis (a box filter)
constexpr int SHADOW_MAX = 512;        // the shadow mask's largest side, texels
constexpr int FAR_MAX = 256;           // the far slices' largest side, texels
constexpr double SHADOW_SOFT_M = 40.0; // the shadow's softening, metres (blur passes x texel)
// Cell draws sit apart from the variant maps' (element ids 0..variants) on the cloud stream.
constexpr std::uint64_t CELL_SALT = 1ull << 52;

// Soften a square patch: `passes` rounds of a separable [1 4 6 4 1] / 16 blur (each a variance
// of one texel), clamped at the edges. A cloud's shadow is soft: its edges are thin cloud.
void soften(std::vector<float> &a, int n, int passes) {
  std::vector<float> b(a.size());
  auto at = [n](int v) { return std::min(n - 1, std::max(0, v)); };
  for (int p = 0; p < passes; ++p)
    for (int axis = 0; axis < 2; ++axis) {
      for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x) {
          auto v = [&](int d) {
            return axis == 0 ? a[std::size_t(y) * n + at(x + d)]
                             : a[std::size_t(at(y + d)) * n + x];
          };
          b[std::size_t(y) * n + x] =
              (v(-2) + 4.0f * v(-1) + 6.0f * v(0) + 4.0f * v(1) + v(2)) / 16.0f;
        }
      a.swap(b);
    }
}

std::uint64_t cell_el(int i, int j) { return lsys::cell_id(i, j) ^ CELL_SALT; }

int wrap(long v, int n) { return static_cast<int>(((v % n) + n) % n); }

// Trilinear sample of an n*n*nz map spanning [-extent, extent]^2 x [base, top]; 0 outside.
double sample_map(const std::vector<float> &f, int n, int nz, double extent, double base,
                  double top, double x, double y, double z) {
  const double fx = (x + extent) / (2.0 * extent) * (n - 1);
  const double fy = (y + extent) / (2.0 * extent) * (n - 1);
  const double fz = (z - base) / (top - base) * (nz - 1);
  if (fx < 0 || fx > n - 1 || fy < 0 || fy > n - 1 || fz < 0 || fz > nz - 1)
    return 0.0;
  const int x0 = static_cast<int>(fx), y0 = static_cast<int>(fy), z0 = static_cast<int>(fz);
  const int x1 = std::min(x0 + 1, n - 1), y1 = std::min(y0 + 1, n - 1),
            z1 = std::min(z0 + 1, nz - 1);
  const double tx = fx - x0, ty = fy - y0, tz = fz - z0;
  auto V = [&](int xx, int yy, int zz) { return double(f[lsys::cloud_index(n, xx, yy, zz)]); };
  const double c00 = V(x0, y0, z0) * (1 - tx) + V(x1, y0, z0) * tx;
  const double c10 = V(x0, y1, z0) * (1 - tx) + V(x1, y1, z0) * tx;
  const double c01 = V(x0, y0, z1) * (1 - tx) + V(x1, y0, z1) * tx;
  const double c11 = V(x0, y1, z1) * (1 - tx) + V(x1, y1, z1) * tx;
  return (c00 * (1 - ty) + c10 * ty) * (1 - tz) + (c01 * (1 - ty) + c11 * ty) * tz;
}
} // namespace

cloud_layer::cloud_layer(const cloud_layer_params &p) : m_p(p) {
  m_p.variants = std::max(1, m_p.variants);
  m_p.n = std::max(2, m_p.n);
  m_p.nz = std::max(2, m_p.nz);
  m_p.extent = std::max(1.0, m_p.extent);
  m_p.top = std::max(m_p.base + 1.0, m_p.top);
  m_p.coverage = std::min(1.0, std::max(0.0, m_p.coverage));
  m_p.far_texels = std::max(2, m_p.far_texels + (m_p.far_texels & 1)); // even: a centred cloud
  m_p.far_layers = std::max(1, m_p.far_layers);
  m_p.sun_el = std::min(89.0, std::max(5.0, m_p.sun_el));
  m_c = m_p.spacing > 0.0 ? m_p.spacing : 2.5 * m_p.extent;
  if (m_p.opacity.size() % 2 != 0)
    m_p.opacity.pop_back();
  m_amax = 0.0;
  for (std::size_t k = 1; k < m_p.opacity.size(); k += 2)
    m_amax = std::max(m_amax, m_p.opacity[k]);
  if (m_p.opacity.empty() || m_amax <= 0.0)
    m_amax = 1.0;

  // The maps: each scaled to its own peak (the grammar grows some variants much fainter than
  // others, and a cell showing one must still hold a cloud), then lumped: clear sky below
  // LUMP_FLOOR, squared above. A crossfade of two is what a cell shows, and its optical depth is
  // the crossfade of theirs.
  const int n = m_p.n, nz = m_p.nz;
  for (int v = 0; v < m_p.variants; ++v) {
    lsys::cloud_params cp;
    cp.seed = p.seed;
    cp.variant = static_cast<std::uint32_t>(v);
    cp.n = n;
    cp.nz = nz;
    cp.half = m_p.extent;
    cp.base = m_p.base;
    cp.top = m_p.top;
    cp.sun_az = m_p.sun_az;
    cp.sun_el = m_p.sun_el;
    cp.depth = m_p.depth;
    std::vector<float> f = lsys::cloud_field(cp);
    // A round taper over the outer part of the footprint. The grammar clamps its turtle at the
    // grid's y edges (it wraps x), so a cloud that wanders there piles puffs against the face,
    // and the grid's own window cuts that pile off in a few voxels: a straight edge along the
    // volume's box. The taper also keeps the clouds from reading square.
    for (int gy = 0; gy < n; ++gy)
      for (int gx = 0; gx < n; ++gx) {
        const double u = 2.0 * gx / (n - 1) - 1.0, v = 2.0 * gy / (n - 1) - 1.0;
        const double t = (std::sqrt(u * u + v * v) - TAPER_FROM) / (1.0 - TAPER_FROM);
        const float w = static_cast<float>(t <= 0.0   ? 1.0
                                           : t >= 1.0 ? 0.0
                                                      : 1.0 - t * t * (3.0 - 2.0 * t));
        for (int gz = 0; gz < nz; ++gz)
          f[lsys::cloud_index(n, gx, gy, gz)] *= w;
      }
    const float peak = f.empty() ? 0.0f : *std::max_element(f.begin(), f.end());
    for (float &x : f) {
      const double r = peak > 0.0f ? x / peak : 0.0;
      const double l = std::min(1.0, std::max(0.0, (r - LUMP_FLOOR) / (1.0 - LUMP_FLOOR)));
      x = static_cast<float>(l * l);
    }
    m_maps.push_back(std::move(f));
  }

  set_area(m_p.area);
}

void cloud_layer::set_area(double area) {
  m_p.area = std::max(0.0, area);
  const int F = m_p.far_texels;
  m_m = std::max(2, static_cast<int>(std::ceil(2.0 * m_p.area / m_c)));
  int s = std::max(2, m_p.shadow_texels + (m_p.shadow_texels & 1));
  while (m_m * s > SHADOW_MAX && s > 2)
    s -= 2;
  const bool retexel = s != m_s || m_tau.empty();
  m_s = s;
  int f = std::min(m_p.far_texels, std::max(4, FAR_MAX / m_m));
  f -= f & 1;
  const bool refar = f != m_f || m_far.empty();
  m_f = f;

  // The cells: exactly round(coverage * cells) hold a cloud (at least one when coverage > 0), the
  // ones whose hashed draw ranks lowest, so a small world is never left clear by chance.
  const int ncell = m_m * m_m;
  m_cells.assign(ncell, cloud_cell());
  std::vector<double> rank(ncell);
  for (int j = 0; j < m_m; ++j)
    for (int i = 0; i < m_m; ++i) {
      cloud_cell &c = m_cells[j * m_m + i];
      const std::uint64_t el = cell_el(i, j);
      c.i = i;
      c.j = j;
      c.jx = static_cast<int>(std::lround(
          lsys::uni(m_p.seed, lsys::stream::cloud, el, 1, -m_p.jitter, m_p.jitter) * F));
      c.jy = static_cast<int>(std::lround(
          lsys::uni(m_p.seed, lsys::stream::cloud, el, 2, -m_p.jitter, m_p.jitter) * F));
      c.cx = (i + 0.5) * m_c + c.jx * m_c / F;
      c.cy = (j + 0.5) * m_c + c.jy * m_c / F;
      c.phase = lsys::uni(m_p.seed, lsys::stream::cloud, el, 3, 0.0, double(m_p.variants));
      rank[j * m_m + i] = lsys::uni(m_p.seed, lsys::stream::cloud, el, 0);
    }
  int want = static_cast<int>(std::lround(m_p.coverage * ncell));
  if (m_p.coverage > 0.0)
    want = std::max(1, want);
  std::vector<int> order(ncell);
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](int a, int b) { return rank[a] < rank[b]; });
  for (int k = 0; k < want; ++k)
    m_cells[order[k]].occupied = true;
  if (retexel)
    build_tau();
  if (refar)
    build_far();
}

void cloud_layer::build_far() {
  // Each map's coarse slices, box-filtered from the map: what a far shell at that height shows.
  m_far.clear();
  const int n = m_p.n, nz = m_p.nz;
  const double ft = m_c / m_f;
  m_far_n = 2 * static_cast<int>(std::ceil(m_p.extent / ft));
  const int L = m_p.far_layers;
  const double dz = (m_p.top - m_p.base) / L;
  for (const std::vector<float> &f : m_maps) {
    std::vector<float> far(static_cast<std::size_t>(m_far_n) * m_far_n * L);
    for (int k = 0; k < L; ++k)
      for (int py = 0; py < m_far_n; ++py)
        for (int px = 0; px < m_far_n; ++px) {
          double acc = 0.0;
          for (int a = 0; a < FAR_SUB; ++a)
            for (int b = 0; b < FAR_SUB; ++b)
              for (int c = 0; c < FAR_SUB; ++c) {
                const double x = (px - m_far_n / 2 + (a + 0.5) / FAR_SUB) * ft;
                const double y = (py - m_far_n / 2 + (b + 0.5) / FAR_SUB) * ft;
                const double z = m_p.base + (k + (c + 0.5) / FAR_SUB) * dz;
                acc += opacity_of(sample_map(f, n, nz, m_p.extent, m_p.base, m_p.top, x, y, z));
              }
          far[(static_cast<std::size_t>(k) * m_far_n + py) * m_far_n + px] =
              static_cast<float>(acc / (FAR_SUB * FAR_SUB * FAR_SUB) / m_amax);
        }
    m_far.push_back(std::move(far));
  }
}

void cloud_layer::build_tau() {
  const int n = m_p.n, nz = m_p.nz;
  m_tau.clear();
  // Each map's optical depth over the base plane. The ray from a base-plane point b climbs the
  // deck toward the sun, travelling `travel` metres sideways, so the patch spans the cloud plus
  // that on every side.
  double sx, sy, sz;
  sun_dir(sx, sy, sz);
  const double path = (m_p.top - m_p.base) / sz;
  const double travel = path * std::sqrt(sx * sx + sy * sy);
  const double ts = m_c / m_s;
  m_tau_n = 2 * static_cast<int>(std::ceil((m_p.extent + travel) / ts));
  const double ds = path / TAU_STEPS;
  for (const std::vector<float> &f : m_maps) {
    std::vector<float> tau(static_cast<std::size_t>(m_tau_n) * m_tau_n);
    for (int py = 0; py < m_tau_n; ++py) {
      const double by = (py - m_tau_n / 2 + 0.5) * ts;
      for (int px = 0; px < m_tau_n; ++px) {
        const double bx = (px - m_tau_n / 2 + 0.5) * ts;
        double t = 0.0;
        for (int s = 0; s < TAU_STEPS; ++s) {
          const double l = (s + 0.5) * ds;
          t += opacity_of(sample_map(f, n, nz, m_p.extent, m_p.base, m_p.top, bx + l * sx,
                                     by + l * sy, m_p.base + l * sz));
        }
        tau[static_cast<std::size_t>(py) * m_tau_n + px] = static_cast<float>(t * ds);
      }
    }
    soften(tau, m_tau_n, std::max(1, static_cast<int>(std::lround(SHADOW_SOFT_M / ts))));
    m_tau.push_back(std::move(tau));
  }
}

double cloud_layer::opacity_of(double v) const {
  const std::vector<double> &t = m_p.opacity;
  if (t.empty())
    return v;
  if (v <= t[0])
    return t[1];
  for (std::size_t k = 2; k + 1 < t.size(); k += 2)
    if (v <= t[k]) {
      const double u = t[k] > t[k - 2] ? (v - t[k - 2]) / (t[k] - t[k - 2]) : 1.0;
      return t[k - 1] + u * (t[k + 1] - t[k - 1]);
    }
  return t[t.size() - 1];
}

int cloud_layer::occupied() const {
  int k = 0;
  for (const cloud_cell &c : m_cells)
    k += c.occupied ? 1 : 0;
  return k;
}

void cloud_layer::sun_dir(double &x, double &y, double &z) const {
  const double az = m_p.sun_az * kPi / 180.0, el = m_p.sun_el * kPi / 180.0;
  x = std::cos(el) * std::sin(az);
  y = -std::cos(el) * std::cos(az);
  z = std::sin(el);
}

void cloud_layer::base_point(double x, double y, double z, double &bx, double &by) const {
  double sx, sy, sz;
  sun_dir(sx, sy, sz);
  const double l = (m_p.base - z) / sz;
  bx = x + l * sx;
  by = y + l * sy;
}

int cloud_layer::cell_variant(const cloud_cell &c, long k) const {
  const int V = m_p.variants;
  if (V == 1)
    return 0;
  const std::uint64_t el = cell_el(c.i, c.j);
  const int start = lsys::irand(m_p.seed, lsys::stream::cloud, el, 4, 0, V - 1);
  int step = lsys::irand(m_p.seed, lsys::stream::cloud, el, 5, 1, V - 1);
  while (std::gcd(step, V) != 1) // a step coprime with V visits every variant, never one twice
    step = step % (V - 1) + 1;
  return wrap(start + static_cast<long>(step) * (k % V), V);
}

cloud_layer::morph_state cloud_layer::cell_morph(const cloud_cell &c, double m) const {
  const double mm = m + c.phase;
  const long k = static_cast<long>(std::floor(mm));
  double u = mm - std::floor(mm);
  morph_state s;
  s.a = cell_variant(c, k);
  s.b = cell_variant(c, k + 1);
  s.u = u * u * (3.0 - 2.0 * u);
  return s;
}

void cloud_layer::cell_field(const cloud_cell &c, double m, std::vector<float> &out) const {
  const morph_state s = cell_morph(c, m);
  const std::vector<float> &A = m_maps[s.a], &B = m_maps[s.b];
  out.resize(A.size());
  const float u = static_cast<float>(s.u), w = 1.0f - u;
  for (std::size_t k = 0; k < A.size(); ++k)
    out[k] = w * A[k] + u * B[k];
}

void cloud_layer::add_shadow(double m, std::size_t first, std::size_t count,
                             std::vector<float> &tau) const {
  const int N = shadow_size();
  tau.resize(static_cast<std::size_t>(N) * N, 0.0f);
  const double ts = m_c / m_s;
  const std::size_t last = std::min(m_cells.size(), first + count);
  for (std::size_t ci = first; ci < last; ++ci) {
    const cloud_cell &c = m_cells[ci];
    if (!c.occupied)
      continue;
    const morph_state s = cell_morph(c, m);
    const std::vector<float> &A = m_tau[s.a], &B = m_tau[s.b];
    const float u = static_cast<float>(s.u), w = 1.0f - u;
    // The cloud's centre, to the nearest texel corner.
    const long ox = std::lround(c.cx / ts) - m_tau_n / 2;
    const long oy = std::lround(c.cy / ts) - m_tau_n / 2;
    for (int py = 0; py < m_tau_n; ++py) {
      float *row = &tau[static_cast<std::size_t>(wrap(oy + py, N)) * N];
      const std::size_t pr = static_cast<std::size_t>(py) * m_tau_n;
      for (int px = 0; px < m_tau_n; ++px)
        row[wrap(ox + px, N)] += w * A[pr + px] + u * B[pr + px];
    }
  }
}

void cloud_layer::shade_rows(const std::vector<float> &tau, double k, double floor, int row0,
                             int rows, std::vector<unsigned char> &out) const {
  const int N = shadow_size();
  out.resize(static_cast<std::size_t>(N) * N);
  floor = std::min(1.0, std::max(0.0, floor));
  const std::size_t a = static_cast<std::size_t>(std::max(0, row0)) * N;
  const std::size_t b = static_cast<std::size_t>(std::min(N, row0 + rows)) * N;
  for (std::size_t i = a; i < b && i < tau.size(); ++i) {
    const double s = floor + (1.0 - floor) * std::exp(-k * tau[i]);
    out[i] = static_cast<unsigned char>(std::lround(std::min(1.0, std::max(0.0, s)) * 255.0));
  }
}

void cloud_layer::shadow_tau(double m, std::vector<float> &out) const {
  out.clear();
  add_shadow(m, 0, m_cells.size(), out);
}

void cloud_layer::shadow_mask(double m, double k, double floor,
                              std::vector<unsigned char> &out) const {
  std::vector<float> tau;
  shadow_tau(m, tau);
  shade_rows(tau, k, floor, 0, shadow_size(), out);
}

void cloud_layer::add_far(double m, std::size_t first, std::size_t count,
                          const std::vector<float> &weight, std::vector<float> &acc) const {
  const int N = far_size(), L = m_p.far_layers;
  acc.resize(static_cast<std::size_t>(N) * N * L, 0.0f);
  const double ft = m_c / m_f;
  const std::size_t last = std::min(m_cells.size(), first + count);
  for (std::size_t ci = first; ci < last; ++ci) {
    const cloud_cell &c = m_cells[ci];
    const float wt = ci < weight.size() ? weight[ci] : 1.0f;
    if (!c.occupied || wt <= 0.0f)
      continue;
    const morph_state s = cell_morph(c, m);
    const std::vector<float> &A = m_far[s.a], &B = m_far[s.b];
    const float u = static_cast<float>(s.u) * wt, w = (1.0f - static_cast<float>(s.u)) * wt;
    const long ox = std::lround(c.cx / ft) - m_far_n / 2;
    const long oy = std::lround(c.cy / ft) - m_far_n / 2;
    for (int k = 0; k < L; ++k)
      for (int py = 0; py < m_far_n; ++py) {
        float *row = &acc[(static_cast<std::size_t>(k) * N + wrap(oy + py, N)) * N];
        const std::size_t pr = (static_cast<std::size_t>(k) * m_far_n + py) * m_far_n;
        for (int px = 0; px < m_far_n; ++px)
          row[wrap(ox + px, N)] += w * A[pr + px] + u * B[pr + px];
      }
  }
}

void cloud_layer::far_bytes(const std::vector<float> &acc, int slice0, int slices,
                            std::vector<unsigned char> &out) const {
  out.resize(acc.size());
  const std::size_t plane = static_cast<std::size_t>(far_size()) * far_size();
  const std::size_t a = static_cast<std::size_t>(std::max(0, slice0)) * plane;
  const std::size_t b = std::min(acc.size(), static_cast<std::size_t>(slice0 + slices) * plane);
  for (std::size_t i = a; i < b; ++i)
    out[i] = static_cast<unsigned char>(std::min(1.0f, std::max(0.0f, acc[i])) * 255.0f + 0.5f);
}

void cloud_layer::far_density(double m, const std::vector<float> &weight,
                              std::vector<unsigned char> &out) const {
  std::vector<float> acc;
  add_far(m, 0, m_cells.size(), weight, acc);
  far_bytes(acc, 0, m_p.far_layers, out);
}

} // namespace world
} // namespace cvc
