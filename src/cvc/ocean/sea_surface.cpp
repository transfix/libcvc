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

// sea_surface.cpp — the sea a floating body feels (see sea_surface.h).

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cvc/algo/det_math.h>
#include <cvc/ocean/sea_surface.h>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace cvc {
namespace ocean {

namespace det = ::cvc::algo::det; // the same bits on every platform (det_math.h)

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

double smoothstep(double e0, double e1, double x) {
  if (!(e1 > e0))
    return x >= e1 ? 1.0 : 0.0;
  const double t = std::clamp((x - e0) / (e1 - e0), 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

bool power_of_two(int n) { return n >= 1 && (n & (n - 1)) == 0; }
bool finite(double v) { return std::isfinite(v); }

// FNV-1a over the bytes of every value the surface reads.
struct digester {
  std::uint64_t h = 0xcbf29ce484222325ull;
  void bytes(const void *p, std::size_t n) {
    const auto *b = static_cast<const unsigned char *>(p);
    for (std::size_t i = 0; i < n; ++i) {
      h ^= b[i];
      h *= 0x100000001b3ull;
    }
  }
  void add(double v) { bytes(&v, sizeof v); }
  void add(std::int64_t v) { bytes(&v, sizeof v); }
  void add(const std::vector<float> &v) {
    add(std::int64_t(v.size()));
    if (!v.empty())
      bytes(v.data(), v.size() * sizeof(float));
  }
};

// The four Catmull-Rom weights at fraction t between samples 1 and 2.
void catmull_rom(double t, double w[4]) {
  const double t2 = t * t, t3 = t2 * t;
  w[0] = 0.5 * (-t3 + 2.0 * t2 - t);
  w[1] = 0.5 * (3.0 * t3 - 5.0 * t2 + 2.0);
  w[2] = 0.5 * (-3.0 * t3 + 4.0 * t2 + t);
  w[3] = 0.5 * (t3 - t2);
}

// A 4 x 4 Catmull-Rom stencil on a periodic grid x grid lattice of a tile.
struct stencil {
  int ix[4], iy[4];
  double wx[4], wy[4];
  int grid;

  stencil(double x, double y, double tile, int g) : grid(g) {
    const double fx = x / tile * g, fy = y / tile * g;
    const double flx = std::floor(fx), fly = std::floor(fy);
    catmull_rom(fx - flx, wx);
    catmull_rom(fy - fly, wy);
    // Wrap without overflow for any finite coordinate: reduce the floor modulo g first.
    const long long cx = static_cast<long long>(std::fmod(flx, double(g))),
                    cy = static_cast<long long>(std::fmod(fly, double(g)));
    for (int i = 0; i < 4; ++i) {
      ix[i] = int(((cx - 1 + i) % g + g) % g);
      iy[i] = int(((cy - 1 + i) % g + g) % g);
    }
  }
  double operator()(const std::vector<double> &f) const {
    double s = 0.0;
    for (int j = 0; j < 4; ++j) {
      const double *row = f.data() + std::size_t(iy[j]) * std::size_t(grid);
      s += wy[j] *
           (wx[0] * row[ix[0]] + wx[1] * row[ix[1]] + wx[2] * row[ix[2]] + wx[3] * row[ix[3]]);
    }
    return s;
  }
};

bool same_bits(double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; }

sea_params checked(const sea_params &p) {
  if (!finite(p.level) || !(p.depth_m > 0.0) || !finite(p.depth_m) || !(p.gravity > 0.0) ||
      !finite(p.gravity))
    throw std::invalid_argument("sea_surface: level, depth and gravity must be finite, depth and "
                                "gravity above 0");
  if (!(p.wave_amp >= 0.0) || !finite(p.wave_amp) || !(p.chop >= 0.0) || !finite(p.chop))
    throw std::invalid_argument("sea_surface: wave_amp and chop must be finite and >= 0");
  if (!power_of_two(p.band.modes) || p.band.modes < 2 || !power_of_two(p.band.grid) ||
      p.band.grid < p.band.modes)
    throw std::invalid_argument("sea_surface: band.modes and band.grid must be powers of two, "
                                "2 <= modes <= grid");
  if (p.point_iterations < 0 || p.point_iterations > 64)
    throw std::invalid_argument("sea_surface: point_iterations must be in 0..64");
  if (!(p.shore_limit >= 0.0) || !(p.shore_limit <= 1.0))
    throw std::invalid_argument("sea_surface: shore_limit must be in [0, 1]");
  if (!(p.source_cell_m >= 0.0) || !finite(p.source_cell_m))
    throw std::invalid_argument("sea_surface: source_cell_m must be finite and >= 0");
  return p;
}

} // namespace

struct sea_surface::frame {
  double t = 0.0;
  std::vector<band_frame> bands;
  // Memos of pure functions of this surface at time t, so the order in which reads fill them
  // (and from which threads) changes no value; the mutex guards both:
  // - with source_cell_m, the move x - x0 at the lattice corners found so far, keyed by the
  //   corner's index pair;
  // - the last source found, keyed by its point (source()).
  mutable std::mutex lattice_mutex;
  mutable std::unordered_map<std::uint64_t, std::array<double, 2>> lattice;
  mutable bool last_valid = false;
  mutable double last_x = 0.0, last_y = 0.0;
  mutable std::array<double, 2> last_x0{};
};

sea_surface::sea_surface(std::vector<sea_cascade> cascades, const sea_params &p,
                         std::shared_ptr<const shore_grid> grid,
                         std::shared_ptr<const shore_layer> layer)
    : _p(checked(p)), _fft(_p.band.grid), _grid(std::move(grid)), _layer(std::move(layer)) {
  if (_grid) {
    const shore_grid &g = *_grid;
    if (g.rows < 2 || g.cols < 2 || !(g.cell > 0.0) || !finite(g.cell) || !finite(g.min_x) ||
        !finite(g.min_y) || g.depth.size() != g.count() || g.shore_m.size() != g.count())
      throw std::invalid_argument("sea_surface: the grid must be at least 2 x 2, its cell above "
                                  "0, and its depth and shore planes rows x cols");
  }
  const int M = _p.band.modes, G = _p.band.grid, half = M / 2;
  digester d;
  d.bytes("cvc::ocean::sea_surface v1", 26);
  d.add(_p.level);
  d.add(_p.depth_m);
  d.add(_p.gravity);
  d.add(_p.wave_amp);
  d.add(_p.chop);
  d.add(std::int64_t(M));
  d.add(std::int64_t(G));
  d.add(std::int64_t(_p.point_iterations));
  d.add(_p.shore_limit);
  d.add(std::int64_t((_p.bores ? 1 : 0) | (_p.swash ? 2 : 0)));
  if (_p.source_cell_m > 0.0) { // only then, so every exact surface keeps its digest
    d.bytes("source_cell_m", 13);
    d.add(_p.source_cell_m);
  }
  for (const sea_cascade &c : cascades) {
    const int n = c.h0.n;
    if (!power_of_two(n) || n < M || c.h0.texels.size() != std::size_t(n) * n * 4 ||
        !(c.h0.tile_m > 0.0) || !finite(c.h0.tile_m))
      throw std::invalid_argument("sea_surface: a cascade's h0 must be n x n texels, n a power of "
                                  "two of at least band.modes (" +
                                  std::to_string(M) + "), its tile above 0");
    if (!(c.fade_m >= 0.0) || !finite(c.fade_m) || !(c.decay_k >= 0.0) || !finite(c.decay_k))
      throw std::invalid_argument("sea_surface: a cascade's fade and decay must be finite, >= 0");
    cascade_band b;
    b.tile_m = c.h0.tile_m;
    b.fade_m = c.fade_m;
    b.decay_k = c.decay_k;
    // The modes |m| < modes / 2 along each axis: a set closed under k -> -k, so each field the
    // packing reads is real.
    for (int r = 0; r < n; ++r) {
      const int my = r - n / 2;
      if (my <= -half || my >= half)
        continue;
      for (int col = 0; col < n; ++col) {
        const int mx = col - n / 2;
        if (mx <= -half || mx >= half)
          continue;
        const float *tx = c.h0.texel(col, r);
        if (tx[0] == 0.0f && tx[1] == 0.0f && tx[2] == 0.0f && tx[3] == 0.0f)
          continue;
        mode m;
        m.kx = texel_k(col, n, b.tile_m);
        m.ky = texel_k(r, n, b.tile_m);
        const double k = det::hypot(m.kx, m.ky);
        if (k < 1e-6)
          continue; // OceanFFT's time evolution drops it too
        m.knx = m.kx / k;
        m.kny = m.ky / k;
        // OceanFFT's dispersion: sqrt(g k tanh(min(k d, 20))).
        m.omega = std::sqrt(_p.gravity * k * det::tanh(std::min(k * _p.depth_m, 20.0)));
        m.h0r = tx[0];
        m.h0i = tx[1];
        m.hcr = tx[2];
        m.hci = tx[3];
        m.gx = (mx + G) % G;
        m.gy = (my + G) % G;
        b.modes.push_back(m);
      }
    }
    d.add(std::int64_t(n));
    d.add(b.tile_m);
    d.add(b.fade_m);
    d.add(b.decay_k);
    d.add(c.h0.texels);
    _cascades.push_back(std::move(b));
  }
  if (_grid) {
    d.add(std::int64_t(_grid->rows));
    d.add(std::int64_t(_grid->cols));
    d.add(_grid->min_x);
    d.add(_grid->min_y);
    d.add(_grid->cell);
    d.add(_grid->depth);
    d.add(_grid->shore_m);
  }
  if (_layer) {
    const shore_layer &L = *_layer;
    d.add(std::int64_t(L.rows));
    d.add(std::int64_t(L.cols));
    d.add(L.min_x);
    d.add(L.min_y);
    d.add(L.cell);
    d.add(L.omega);
    d.add(L.dry_time_s);
    for (const shore_train &tr : L.trains) {
      d.add(tr.ratio);
      d.add(tr.amplitude);
      d.add(tr.phase);
      d.add(tr.phase_reflected);
    }
    for (const std::vector<float> *f : {&L.phase, &L.amplitude, &L.reflected, &L.steepness,
                                        &L.dir_x, &L.dir_y, &L.wavenumber, &L.breaking, &L.runup})
      d.add(*f);
  }
  _digest = d.h;
}

std::shared_ptr<sea_surface::frame> sea_surface::compute(double t) const {
  auto f = std::make_shared<frame>();
  f->t = t;
  const int G = _p.band.grid;
  const std::size_t cells = std::size_t(G) * std::size_t(G);
  std::vector<double> a1(cells * 2), a2(cells * 2), a3(cells * 2);
  for (const cascade_band &b : _cascades) {
    std::fill(a1.begin(), a1.end(), 0.0);
    std::fill(a2.begin(), a2.end(), 0.0);
    std::fill(a3.begin(), a3.end(), 0.0);
    for (const mode &m : b.modes) {
      const double ph = m.omega * t, c = det::cos(ph), s = det::sin(ph);
      // h = h0 e^(i w t) + conj h0(-k) e^(-i w t), and dh/dt = i w (h0 e^(i w t) - ...).
      const double ar = m.h0r * c - m.h0i * s, ai = m.h0r * s + m.h0i * c;
      const double br = m.hcr * c + m.hci * s, bi = m.hci * c - m.hcr * s;
      const double hr = ar + br, hi = ai + bi;
      const double dr = -m.omega * (ai - bi), di = m.omega * (ar - br);
      const std::size_t i = (std::size_t(m.gy) * std::size_t(G) + std::size_t(m.gx)) * 2;
      // Two real fields to a transform: eta + i w; D_x + i D_y (D = i k/|k| h); u + i v.
      a1[i] += hr - di;
      a1[i + 1] += hi + dr;
      a2[i] += -hi * m.knx - hr * m.kny;
      a2[i + 1] += hr * m.knx - hi * m.kny;
      a3[i] += -di * m.knx - dr * m.kny;
      a3[i + 1] += dr * m.knx - di * m.kny;
    }
    _fft.run_2d(a1.data(), +1);
    _fft.run_2d(a2.data(), +1);
    _fft.run_2d(a3.data(), +1);
    band_frame out;
    out.grid = G;
    out.tile_m = b.tile_m;
    for (std::vector<double> *v : {&out.eta, &out.dx, &out.dy, &out.u, &out.v, &out.w})
      v->resize(cells);
    for (std::size_t i = 0; i < cells; ++i) {
      out.eta[i] = a1[i * 2];
      out.w[i] = a1[i * 2 + 1];
      out.dx[i] = a2[i * 2];
      out.dy[i] = a2[i * 2 + 1];
      out.u[i] = a3[i * 2];
      out.v[i] = a3[i * 2 + 1];
    }
    f->bands.push_back(std::move(out));
  }
  return f;
}

std::shared_ptr<const sea_surface::frame> sea_surface::frame_at(double t) const {
  {
    std::lock_guard<std::mutex> lock(_cache_mutex);
    if (_cache && same_bits(_cache->t, t))
      return _cache;
  }
  std::shared_ptr<const frame> f = compute(t); // outside the lock: a pure function of t
  std::lock_guard<std::mutex> lock(_cache_mutex);
  _cache = f;
  return f;
}

double sea_surface::grid_sample(const std::vector<float> &f, double x, double y) const {
  const shore_grid &g = *_grid;
  const double fc = std::clamp((x - g.min_x) / g.cell, 0.0, double(g.cols - 1));
  const double fr = std::clamp((y - g.min_y) / g.cell, 0.0, double(g.rows - 1));
  const int c0 = std::min(g.cols - 2, int(std::floor(fc))),
            r0 = std::min(g.rows - 2, int(std::floor(fr)));
  const double u = fc - c0, v = fr - r0;
  const std::size_t i = std::size_t(r0) * std::size_t(g.cols) + std::size_t(c0);
  const double a = f[i], b = f[i + 1], c = f[i + std::size_t(g.cols)],
               d = f[i + std::size_t(g.cols) + 1];
  return (a * (1 - u) + b * u) * (1 - v) + (c * (1 - u) + d * u) * v;
}

double sea_surface::depth(double x, double y) const {
  return _grid ? grid_sample(_grid->depth, x, y) : kInf;
}

std::array<double, 2> sea_surface::move(const frame &f, double x, double y, double t) const {
  const double d = depth(x, y);
  double dx = 0.0, dy = 0.0;
  for (std::size_t c = 0; c < _cascades.size(); ++c) {
    const cascade_band &b = _cascades[c];
    const double fade = !(d > 0.0) ? 0.0 : b.fade_m > 0.0 ? smoothstep(0.0, b.fade_m, d) : 1.0;
    if (!(fade > 0.0))
      continue;
    const band_frame &bf = f.bands[c];
    const stencil st(x, y, bf.tile_m, bf.grid);
    dx += fade * st(bf.dx);
    dy += fade * st(bf.dy);
  }
  dx *= _p.chop;
  dy *= _p.chop;
  if (_layer) {
    const shore_layer::sample s = _layer->evaluate(x, y, t);
    dx += s.dx;
    dy += s.dy;
  }
  if (_grid) {
    const double lim = _p.shore_limit * std::max(grid_sample(_grid->shore_m, x, y), 0.0);
    const double len = det::hypot(dx, dy);
    if (len > lim) {
      const double s = len > 0.0 ? lim / len : 0.0;
      dx *= s;
      dy *= s;
    }
  }
  return {dx, dy};
}

std::array<double, 2> sea_surface::source(const frame &f, double x, double y, double t) const {
  // The frame's last source. A fluid field reads the height over a cell and then the velocity in
  // it, at the same (x, y) and time, so the second read finds its source here. A frame is this
  // surface at time t, so the point alone is the key. It lives in the frame rather than in a
  // thread_local, so the library holds no process-wide state.
  {
    std::lock_guard<std::mutex> lock(f.lattice_mutex);
    if (f.last_valid && same_bits(f.last_x, x) && same_bits(f.last_y, y))
      return f.last_x0;
  }
  const std::array<double, 2> x0 = find_source(f, x, y, t);
  std::lock_guard<std::mutex> lock(f.lattice_mutex);
  f.last_valid = true;
  f.last_x = x;
  f.last_y = y;
  f.last_x0 = x0;
  return x0;
}

std::array<double, 2> sea_surface::find_source(const frame &f, double x, double y, double t) const {
  const double h = _p.source_cell_m;
  if (!(h > 0.0) || _p.point_iterations <= 0)
    return source_exact(f, x, y, t);
  // The lattice cell around (x, y) and the move x - x0 at its four corners, each found once per
  // time (source_exact at the corner) and kept in the frame.
  const double gx = x / h, gy = y / h;
  if (!(std::fabs(gx) < 1e9) || !(std::fabs(gy) < 1e9)) // past the keys' range: no lattice
    return source_exact(f, x, y, t);
  const double fi = std::floor(gx), fj = std::floor(gy);
  const double u = gx - fi, v = gy - fj;
  const auto i0 = static_cast<std::int64_t>(fi), j0 = static_cast<std::int64_t>(fj);
  const auto key = [](std::int64_t i, std::int64_t j) {
    return (std::uint64_t(std::uint32_t(i)) << 32) | std::uint64_t(std::uint32_t(j));
  };
  std::array<double, 2> o[4];
  bool have[4] = {false, false, false, false};
  {
    std::lock_guard<std::mutex> lock(f.lattice_mutex);
    for (int k = 0; k < 4; ++k) {
      const auto it = f.lattice.find(key(i0 + (k & 1), j0 + (k >> 1)));
      if (it != f.lattice.end()) {
        o[k] = it->second;
        have[k] = true;
      }
    }
  }
  for (int k = 0; k < 4; ++k) {
    if (have[k])
      continue;
    const std::int64_t i = i0 + (k & 1), j = j0 + (k >> 1);
    const double cx = double(i) * h, cy = double(j) * h;
    const std::array<double, 2> p = source_exact(f, cx, cy, t);
    o[k] = {cx - p[0], cy - p[1]};
    std::lock_guard<std::mutex> lock(f.lattice_mutex);
    f.lattice.emplace(key(i, j), o[k]);
  }
  const double ox =
      (o[0][0] * (1.0 - u) + o[1][0] * u) * (1.0 - v) + (o[2][0] * (1.0 - u) + o[3][0] * u) * v;
  const double oy =
      (o[0][1] * (1.0 - u) + o[1][1] * u) * (1.0 - v) + (o[2][1] * (1.0 - u) + o[3][1] * u) * v;
  double px = x - ox, py = y - oy;
  // Then one Newton step on F(p) = p + D(p) - (x, y), one evaluation of D. The interpolated move
  // m(x) = x - x0(x) gives F's inverse Jacobian at the solution, dx0/dx = I - dm/dx, so the
  // lattice supplies the step's matrix too. The step is skipped when it is longer than a cell (a
  // fold, where the corners disagree on the branch).
  const std::array<double, 2> D = move(f, px, py, t);
  const double rx = px + D[0] - x, ry = py + D[1] - y;
  const double mxx = ((o[1][0] - o[0][0]) * (1.0 - v) + (o[3][0] - o[2][0]) * v) / h;
  const double mxy = ((o[2][0] - o[0][0]) * (1.0 - u) + (o[3][0] - o[1][0]) * u) / h;
  const double myx = ((o[1][1] - o[0][1]) * (1.0 - v) + (o[3][1] - o[2][1]) * v) / h;
  const double myy = ((o[2][1] - o[0][1]) * (1.0 - u) + (o[3][1] - o[1][1]) * u) / h;
  const double sx = (1.0 - mxx) * rx - mxy * ry, sy = -myx * rx + (1.0 - myy) * ry;
  if (sx * sx + sy * sy <= h * h) {
    px -= sx;
    py -= sy;
  }
  return {px, py};
}

std::array<double, 2> sea_surface::source_exact(const frame &f, double x, double y,
                                                double t) const {
  // Solve F(p) = p + D(p) - (x, y) = 0 by Broyden's method: the first step is the fixed point
  // p <- (x, y) - D(p); each later step uses the secant estimate of F's Jacobian built from the
  // steps so far, one evaluation of D per step, as the fixed point costs. Where the plain fixed
  // point contracts only by |dD/dp| per step (slowly on steep, choppy water), this converges
  // superlinearly. A step that grows the residual restarts the estimate at the identity.
  const int n = _p.point_iterations;
  if (n <= 0)
    return {x, y};
  double px = x, py = y;
  std::array<double, 2> D = move(f, px, py, t);
  double fx = D[0], fy = D[1];                       // F at p = (x, y)
  double j00 = 1.0, j01 = 0.0, j10 = 0.0, j11 = 1.0; // the Jacobian estimate
  for (int k = 0; k < n; ++k) {
    const double det2 = j00 * j11 - j01 * j10;
    if (!(std::fabs(det2) > 1e-9)) {
      j00 = j11 = 1.0;
      j01 = j10 = 0.0;
    }
    const double dt = j00 * j11 - j01 * j10;
    const double sx = -(j11 * fx - j01 * fy) / dt, sy = -(-j10 * fx + j00 * fy) / dt;
    const double nx = px + sx, ny = py + sy;
    if (k == n - 1)
      return {nx, ny};
    D = move(f, nx, ny, t);
    const double gx = nx + D[0] - x, gy = ny + D[1] - y;
    if (gx * gx + gy * gy > fx * fx + fy * fy) { // worse: back to the identity
      j00 = j11 = 1.0;
      j01 = j10 = 0.0;
    } else {
      const double ss = sx * sx + sy * sy;
      if (ss > 0.0) { // J += (dF - J s) s^T / (s . s)
        const double rx = (gx - fx) - (j00 * sx + j01 * sy);
        const double ry = (gy - fy) - (j10 * sx + j11 * sy);
        j00 += rx * sx / ss;
        j01 += rx * sy / ss;
        j10 += ry * sx / ss;
        j11 += ry * sy / ss;
      }
    }
    px = nx;
    py = ny;
    fx = gx;
    fy = gy;
  }
  return {px, py};
}

std::array<double, 2> sea_surface::source(double x, double y, double t) const {
  return source(*frame_at(t), x, y, t);
}

std::array<double, 2> sea_surface::displacement(double x, double y, double t) const {
  return move(*frame_at(t), x, y, t);
}

double sea_surface::height(double x, double y, double t) const {
  if (_grid) {
    const double d = depth(x, y);
    if (std::isnan(d))
      return _p.level; // off the grid's world: no water
    if (!(d > 0.0)) {  // dry ground: the swash, or nothing
      if (_p.swash && _layer)
        return _p.level + _layer->swash(x, y, t, _p.level - d, _p.level).front;
      return _p.level;
    }
  }
  const std::shared_ptr<const frame> f = frame_at(t);
  const std::array<double, 2> x0 = source(*f, x, y, t);
  const double d0 = depth(x0[0], x0[1]);
  double h = 0.0;
  for (std::size_t c = 0; c < _cascades.size(); ++c) {
    const cascade_band &b = _cascades[c];
    const double fade = !(d0 > 0.0) ? 0.0 : b.fade_m > 0.0 ? smoothstep(0.0, b.fade_m, d0) : 1.0;
    if (!(fade > 0.0))
      continue;
    const band_frame &bf = f->bands[c];
    h += fade * stencil(x0[0], x0[1], bf.tile_m, bf.grid)(bf.eta);
  }
  h *= _p.wave_amp;
  if (_layer)
    h += _layer->evaluate(x0[0], x0[1], t).eta;
  return _p.level + h;
}

std::array<double, 3> sea_surface::velocity(double x, double y, double z, double t) const {
  if (_grid) {
    const double d = depth(x, y);
    if (std::isnan(d))
      return {0.0, 0.0, 0.0};
    if (!(d > 0.0)) { // dry ground: the swash runs up and drains along the slope
      if (!_p.swash || !_layer)
        return {0.0, 0.0, 0.0};
      const double ground = _p.level - d;
      const shore_layer::swash_sample s = _layer->swash(x, y, t, ground, _p.level);
      if (!(_p.level + s.front > ground))
        return {0.0, 0.0, 0.0}; // no film here now
      const double hstep = _grid->cell;
      // Uphill is where the depth falls: -grad depth.
      const double gx = (depth(x + hstep, y) - depth(x - hstep, y)) / (2.0 * hstep);
      const double gy = (depth(x, y + hstep) - depth(x, y - hstep)) / (2.0 * hstep);
      if (!std::isfinite(gx) || !std::isfinite(gy))
        return {0.0, 0.0, 0.0};
      const double g0 = det::hypot(gx, gy), slope = std::max(g0, 0.02);
      const double ux = g0 > 0.0 ? -gx / g0 : 0.0, uy = g0 > 0.0 ? -gy / g0 : 0.0;
      const double R = _layer->sample_field(_layer->runup, x, y);
      const double cap = std::sqrt(2.0 * _p.gravity * std::max(R, 0.0));
      const double speed = std::clamp(s.rate / slope, -cap, cap);
      return {speed * ux, speed * uy, speed * slope};
    }
  }
  const std::shared_ptr<const frame> f = frame_at(t);
  const std::array<double, 2> x0 = source(*f, x, y, t);
  const double d0 = depth(x0[0], x0[1]);
  const double zrel = z - _p.level, zc = std::min(zrel, 0.0);
  double u = 0.0, v = 0.0, w = 0.0;
  for (std::size_t c = 0; c < _cascades.size(); ++c) {
    const cascade_band &b = _cascades[c];
    const double fade = !(d0 > 0.0) ? 0.0 : b.fade_m > 0.0 ? smoothstep(0.0, b.fade_m, d0) : 1.0;
    if (!(fade > 0.0))
      continue;
    const double decay = b.decay_k > 0.0 ? det::exp(b.decay_k * zc) : 1.0;
    const band_frame &bf = f->bands[c];
    const stencil st(x0[0], x0[1], bf.tile_m, bf.grid);
    u += fade * decay * st(bf.u);
    v += fade * decay * st(bf.v);
    w += fade * decay * st(bf.w);
  }
  u *= _p.chop;
  v *= _p.chop;
  w *= _p.wave_amp;
  if (_layer) {
    const double layerDepth = _grid ? d0 : _p.depth_m;
    const shore_layer::flow fl = _layer->velocity(x0[0], x0[1], zrel, layerDepth, t, _p.bores);
    u += fl.u;
    v += fl.v;
    w += fl.w;
  }
  return {u, v, w};
}

sea_surface::band_frame sea_surface::band(std::size_t c, double t) const {
  if (c >= _cascades.size())
    throw std::out_of_range("sea_surface::band: no cascade " + std::to_string(c));
  return frame_at(t)->bands[c];
}

} // namespace ocean
} // namespace cvc
