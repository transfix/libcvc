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

// cvc-lod-bake -- build a mesh's LOD pyramid and cache it in a scene.cvch5.
//
//   cvc-lod-bake INPUT.{off,raw} OUTPUT.cvch5 [--name NAME] [--rungs N]
//                [--ratio R] [--force]
//
// INPUT is read with cvc::geometry (OFF / cvc-raw). A pyramid is decimated with
// the app compute pool and written under NAME (default: the input basename),
// skipping the work when an up-to-date pyramid is already present unless --force.
// Model formats (glTF/OBJ) are baked from code via cvc::lod::bake_mesh_asset on a
// mesh the caller has already loaded (that is how the demo bundle bake works);
// this CLI stays hermetic and covers the native mesh formats. Requires HDF5.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/lod/pyramid.h>
#include <cvc/lod/store.h>
#include <string>

namespace {

std::string basename_noext(const std::string &path) {
  std::size_t slash = path.find_last_of("/\\");
  std::string base = (slash == std::string::npos) ? path : path.substr(slash + 1);
  std::size_t dot = base.find_last_of('.');
  return (dot == std::string::npos) ? base : base.substr(0, dot);
}

void usage() {
  std::fprintf(stderr, "usage: cvc-lod-bake INPUT.{off,raw} OUTPUT.cvch5 "
                       "[--name NAME] [--rungs N] [--ratio R] [--force]\n");
}

} // namespace

int main(int argc, char **argv) {
  std::string input, output, name;
  cvc::lod::pyramid_params pp;
  bool force = false;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
    if (a == "-h" || a == "--help") {
      usage();
      return 0;
    } else if (a == "--name") {
      name = next();
    } else if (a == "--rungs") {
      pp.max_rungs = std::atoi(next().c_str());
    } else if (a == "--ratio") {
      pp.mesh_ratio = std::atof(next().c_str());
    } else if (a == "--force") {
      force = true;
    } else if (input.empty()) {
      input = a;
    } else if (output.empty()) {
      output = a;
    } else {
      std::fprintf(stderr, "cvc-lod-bake: unexpected argument '%s'\n", a.c_str());
      usage();
      return 2;
    }
  }
  if (input.empty() || output.empty()) {
    usage();
    return 2;
  }
  if (name.empty())
    name = basename_noext(input);

  cvc::app app;
  cvc::geometry g(app);
  try {
    g.read(input);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "cvc-lod-bake: cannot read '%s': %s\n", input.c_str(), e.what());
    return 1;
  }
  if (g.num_tris() == 0) {
    std::fprintf(stderr, "cvc-lod-bake: '%s' has no triangles\n", input.c_str());
    return 1;
  }

  try {
    const bool baked =
        cvc::lod::bake_mesh_asset(app, output, name, g, pp, force, &app.computePool());
    std::printf("%s '%s' (%llu tris) -> %s\n", baked ? "baked" : "up to date, skipped",
                name.c_str(), static_cast<unsigned long long>(g.num_tris()), output.c_str());
  } catch (const std::exception &e) {
    std::fprintf(stderr, "cvc-lod-bake: bake failed: %s\n", e.what());
    return 1;
  }
  return 0;
}
