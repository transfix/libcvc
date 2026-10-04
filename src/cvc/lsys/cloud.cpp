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

// cloud.cpp — the 3-D L-system cloud density field (see cloud.h). Ported in shape from the demo;
// the only change is the few initial mt19937 draws, now hashed on stream::cloud (§5.2).

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cvc/lsys/cloud.h>
#include <cvc/lsys/rng.h>
#include <deque>
#include <vector>

namespace cvc {
namespace lsys {
namespace {

// The cumulus grammar (verbatim). A rose of upward shoots, each recursively frayed into puffs.
constexpr double CLOUD_TURN = 32.0, CLOUD_STEP0 = 8.1, CLOUD_STEP_DECAY = 0.9;
constexpr double CLOUD_PUFF0 = 8.8, CLOUD_PUFF_DECAY = 0.88;
const char *CLOUD_AXIOM = "[A][+++++A][-----A][++++++++++A][----------A][+++++++++++++++A]";
const char *cloudRule(char c) {
  switch (c) {
  case 'A':
    return "FF[+<B]^F[-<C]<F[+<C]vFA";
  case 'B':
    return "F[+<F]F<[-<F]vB";
  case 'C':
    return "^<F[+<F][-<F]^<FC";
  default:
    return nullptr;
  }
}

double percentileSorted(const std::vector<float> &a, double q) {
  if (a.empty())
    return 0.0;
  const double rank = (q / 100.0) * (a.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(std::floor(rank));
  if (lo + 1 >= a.size())
    return a.back();
  return a[lo] + (rank - lo) * (a[lo + 1] - a[lo]);
}

// A pure integer-hash value noise (deterministic; NOT a stateful generator — fine under §5.2).
double vhash3(int x, int y, int z) {
  unsigned h = static_cast<unsigned>(x * 374761393 + y * 668265263 + z * 1274126177);
  h = (h ^ (h >> 13)) * 1274126177u;
  return ((h ^ (h >> 16)) & 0xffffffu) / double(0x1000000);
}
double vnoise3(double x, double y, double z) {
  const int xi = (int)std::floor(x), yi = (int)std::floor(y), zi = (int)std::floor(z);
  double fx = x - xi, fy = y - yi, fz = z - zi;
  auto sm = [](double t) { return t * t * (3.0 - 2.0 * t); };
  fx = sm(fx);
  fy = sm(fy);
  fz = sm(fz);
  auto L = [](double a, double b, double t) { return a + (b - a) * t; };
  const double x00 = L(vhash3(xi, yi, zi), vhash3(xi + 1, yi, zi), fx);
  const double x10 = L(vhash3(xi, yi + 1, zi), vhash3(xi + 1, yi + 1, zi), fx);
  const double x01 = L(vhash3(xi, yi, zi + 1), vhash3(xi + 1, yi, zi + 1), fx);
  const double x11 = L(vhash3(xi, yi + 1, zi + 1), vhash3(xi + 1, yi + 1, zi + 1), fx);
  return L(L(x00, x10, fy), L(x01, x11, fy), fz);
}
double fbm3(double x, double y, double z, int octaves) {
  double f = 0.0, amp = 0.5, tot = 0.0, fr = 1.0;
  for (int i = 0; i < octaves; ++i) {
    f += amp * vnoise3(x * fr, y * fr, z * fr);
    tot += amp;
    amp *= 0.5;
    fr *= 2.02;
  }
  return f / tot;
}
struct V3 {
  double x, y, z;
};
V3 sunDir(double azDeg, double elDeg) {
  const double az = azDeg * M_PI / 180.0, el = elDeg * M_PI / 180.0;
  return {std::cos(el) * std::sin(az), -std::cos(el) * std::cos(az), std::sin(el)};
}

} // namespace

std::vector<float> cloud_field(const cloud_params &p) {
  const int N = p.n, NZ = p.nz;
  if (N <= 0 || NZ <= 0)
    return {};
  auto skyIdx = [N](int z, int y, int x) { return cloud_index(N, x, y, z); };
  std::vector<float> field(static_cast<std::size_t>(NZ) * N * N, 0.0f);

  const double zscale = (double(N) / NZ) * ((p.top - p.base) / (2.0 * p.half));
  // The few initial draws: hashed on stream::cloud, keyed by the variant (so distinct maps
  // decorrelate), where the predecessor stepped a std::mt19937.
  double x = uni(p.seed, stream::cloud, p.variant, 0, 0.25, 0.75) * N;
  double y = uni(p.seed, stream::cloud, p.variant, 1, 0.3, 0.7) * N;
  double z = NZ * 0.42;
  double head = uni(p.seed, stream::cloud, p.variant, 2, 0.0, 360.0);
  double step = CLOUD_STEP0, puff = CLOUD_PUFF0, climb = 0.0;
  int depth = 0;
  struct St {
    double x, y, z, head, step, puff, climb;
    int depth;
  };
  std::vector<St> stack;
  std::deque<char> todo(CLOUD_AXIOM, CLOUD_AXIOM + std::strlen(CLOUD_AXIOM));
  int guard = 0;
  while (!todo.empty() && guard < 4000) {
    ++guard;
    const char c = todo.front();
    todo.pop_front();
    if (c == 'F') {
      x = std::fmod(x + step * std::cos(head * M_PI / 180.0), double(N));
      if (x < 0)
        x += N;
      y = std::min(std::max(y + step * std::sin(head * M_PI / 180.0), 0.0), double(N - 1));
      z = std::min(std::max(z + climb, 1.0), double(NZ - 2));
      const double p2 = 2.0 * puff * puff;
      for (int gz = 0; gz < NZ; ++gz) {
        const double dz = (gz - z) * zscale, dz2 = dz * dz;
        for (int gy = 0; gy < N; ++gy) {
          const double dy = gy - y, dyz2 = dy * dy + dz2;
          for (int gx = 0; gx < N; ++gx) {
            double dx = std::fabs(gx - x);
            dx = std::min(dx, double(N) - dx);
            field[skyIdx(gz, gy, gx)] += static_cast<float>(std::exp(-(dx * dx + dyz2) / p2));
          }
        }
      }
    } else if (c == '+') {
      head += CLOUD_TURN;
    } else if (c == '-') {
      head -= CLOUD_TURN;
    } else if (c == '<') {
      puff *= CLOUD_PUFF_DECAY;
    } else if (c == '^') {
      climb += 0.55;
    } else if (c == 'v') {
      climb -= 0.45;
    } else if (c == '[') {
      stack.push_back({x, y, z, head, step, puff, climb, depth});
    } else if (c == ']') {
      if (!stack.empty()) {
        const St s = stack.back();
        stack.pop_back();
        x = s.x;
        y = s.y;
        z = s.z;
        head = s.head;
        step = s.step;
        puff = s.puff;
        climb = s.climb;
        depth = s.depth;
      }
    } else if (cloudRule(c) && depth < p.depth) {
      ++depth;
      step *= CLOUD_STEP_DECAY;
      const char *r = cloudRule(c);
      todo.insert(todo.begin(), r, r + std::strlen(r));
    }
  }

  std::vector<float> sorted(field);
  std::sort(sorted.begin(), sorted.end());
  const double m = percentileSorted(sorted, 99.9);
  if (m > 0)
    for (float &v : field)
      v = std::min(1.0f, std::max(0.0f, v / static_cast<float>(m)));

  std::vector<double> wf(N), zf(NZ);
  for (int i = 0; i < N; ++i)
    wf[i] = std::sqrt(0.5 - 0.5 * std::cos(2.0 * M_PI * i / (N - 1)));
  for (int k = 0; k < NZ; ++k)
    zf[k] = std::sin(0.12 + (M_PI - 0.24) * k / (NZ - 1));
  for (int gz = 0; gz < NZ; ++gz)
    for (int gy = 0; gy < N; ++gy)
      for (int gx = 0; gx < N; ++gx) {
        float &v = field[skyIdx(gz, gy, gx)];
        v *= static_cast<float>(std::min(wf[gy], wf[gx]) * zf[gz]);
        if (v > 0.0f) {
          const double d = fbm3(gx * 0.30, gy * 0.30, gz * 0.62, 5);
          v = static_cast<float>(std::min(1.0, std::max(0.0, v * (0.32 + 1.5 * d))));
        }
      }

  { // cheap baked top-light: thin each voxel by the cloud density toward the sun
    const V3 sd = sunDir(p.sun_az, p.sun_el);
    double ux = sd.x * N / (2.0 * p.half), uy = sd.y * N / (2.0 * p.half),
           uz = sd.z * NZ / (p.top - p.base);
    const double ul = std::sqrt(ux * ux + uy * uy + uz * uz);
    if (ul > 0) {
      ux /= ul;
      uy /= ul;
      uz /= ul;
    }
    auto samp = [&](double cx, double cy, double cz) -> double {
      if (cx < 0 || cx > N - 1 || cy < 0 || cy > N - 1 || cz < 0 || cz > NZ - 1)
        return 0.0;
      const int x0 = (int)cx, y0 = (int)cy, z0 = (int)cz;
      const int x1 = std::min(x0 + 1, N - 1), y1 = std::min(y0 + 1, N - 1),
                z1 = std::min(z0 + 1, NZ - 1);
      const double tx = cx - x0, ty = cy - y0, tz = cz - z0;
      auto V = [&](int xx, int yy, int zz) { return (double)field[skyIdx(zz, yy, xx)]; };
      const double c00 = V(x0, y0, z0) * (1 - tx) + V(x1, y0, z0) * tx;
      const double c10 = V(x0, y1, z0) * (1 - tx) + V(x1, y1, z0) * tx;
      const double c01 = V(x0, y0, z1) * (1 - tx) + V(x1, y0, z1) * tx;
      const double c11 = V(x0, y1, z1) * (1 - tx) + V(x1, y1, z1) * tx;
      return (c00 * (1 - ty) + c10 * ty) * (1 - tz) + (c01 * (1 - ty) + c11 * ty) * tz;
    };
    const int LSTEPS = 7;
    const double LK = 0.95, LFLOOR = 0.72, LSTEP = 1.6;
    std::vector<float> lit(field.size());
    for (int gz = 0; gz < NZ; ++gz)
      for (int gy = 0; gy < N; ++gy)
        for (int gx = 0; gx < N; ++gx) {
          const std::size_t o = skyIdx(gz, gy, gx);
          const double v = field[o];
          if (v <= 0.0) {
            lit[o] = 0.0f;
            continue;
          }
          double tau = 0.0;
          for (int s = 1; s <= LSTEPS; ++s)
            tau += samp(gx + ux * s * LSTEP, gy + uy * s * LSTEP, gz + uz * s * LSTEP) * LSTEP;
          lit[o] = static_cast<float>(v * (LFLOOR + (1.0 - LFLOOR) * std::exp(-LK * tau)));
        }
    field.swap(lit);
  }
  return field;
}

} // namespace lsys
} // namespace cvc
