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

// cvc-worldgen — headless occupancy/material world generation.
//
//   cvc-worldgen build  --seed N --out DIR [--half 120] [--cell 0.5]
//                       [--preset standard|forest|urban|sparse|island]
//                       [--ontology merged_default] [--scene-kind outdoor]
//                       [--trees N] [--rocks N] [--buildings N] [--amp M]
//                       [--no-preview]
//   cvc-worldgen sample --out DIR --seeds A..B [--half 120] [--cell 0.5] [--preset P]
//   cvc-worldgen inspect <bundle_dir>   # print manifest.json

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/world/bundle.h>
#include <cvc/world/raster.h>
#include <fstream>
#include <sstream>
#include <string>

using namespace cvc::world;

namespace {
const char *opt(int argc, char **argv, const char *name, const char *dflt) {
  for (int i = 0; i < argc - 1; ++i)
    if (std::strcmp(argv[i], name) == 0)
      return argv[i + 1];
  return dflt;
}
bool flag(int argc, char **argv, const char *name) {
  for (int i = 0; i < argc; ++i)
    if (std::strcmp(argv[i], name) == 0)
      return true;
  return false;
}

void apply_preset(world_params &wp, const std::string &preset) {
  scatter_params &s = wp.sc;
  if (preset == "forest") {
    s.tree_count = 120;
    s.rock_count = 16;
    s.building_count = 0;
    wp.hf.amp_m = 10.0;
  } else if (preset == "urban") {
    s.tree_count = 24;
    s.rock_count = 2;
    s.building_count = 16;
    wp.hf.amp_m = 3.0;
  } else if (preset == "sparse") {
    s.tree_count = 12;
    s.rock_count = 6;
    s.building_count = 2;
  } else if (preset == "island") {
    wp.hf.island = true;
    wp.hf.island_peak_m = 40.0;
    wp.hf.island_radius_m = 100.0;
    s.tree_count = 60;
    s.rock_count = 16;
    s.building_count = 3;
  }
  // "standard": scatter defaults + ±120 rolling terrain.
}

int usage() {
  std::fprintf(stderr, "usage: cvc-worldgen <build|sample|inspect> [options] (see --help)\n");
  return 2;
}

int cmd_build(int argc, char **argv) {
  world_params wp;
  const double half = std::atof(opt(argc, argv, "--half", "120"));
  const double cell = std::atof(opt(argc, argv, "--cell", "0.5"));
  wp.min_x = wp.min_y = -half;
  wp.max_x = wp.max_y = half;
  wp.seed = std::strtoull(opt(argc, argv, "--seed", "0"), nullptr, 10);
  wp.ontology = opt(argc, argv, "--ontology", "merged_default");
  apply_preset(wp, opt(argc, argv, "--preset", "standard"));
  if (const char *t = opt(argc, argv, "--trees", nullptr))
    wp.sc.tree_count = std::atoi(t);
  if (const char *r = opt(argc, argv, "--rocks", nullptr))
    wp.sc.rock_count = std::atoi(r);
  if (const char *b = opt(argc, argv, "--buildings", nullptr))
    wp.sc.building_count = std::atoi(b);
  if (const char *a = opt(argc, argv, "--amp", nullptr))
    wp.hf.amp_m = std::atof(a);

  const char *out = opt(argc, argv, "--out", nullptr);
  if (!out) {
    std::fprintf(stderr, "build needs --out DIR\n");
    return 2;
  }

  world_model wm = world_model::generate(wp);
  grid_spec g =
      grid_spec::window(0.5 * (wp.min_x + wp.max_x), 0.5 * (wp.min_y + wp.max_y), half, cell);
  raster_out ro;
  raster(wm, g, ro);

  bundle_options bo;
  bo.scene_kind = opt(argc, argv, "--scene-kind", "outdoor");
  bo.previews = !flag(argc, argv, "--no-preview");
  write_bundle(out, wm, g, ro, bo);

  std::printf("wrote bundle %s\n", out);
  std::printf("  grid %dx%d  cell_w=%.4g m  props=%zu  occupied=%.2f%%\n", g.rows, g.cols,
              g.cell_w(), wm.props().size(), 100.0 * double(ro.occupied_count()) / ro.klass.size());
  return 0;
}

int cmd_sample(int argc, char **argv) {
  const char *out = opt(argc, argv, "--out", nullptr);
  if (!out) {
    std::fprintf(stderr, "sample needs --out DIR\n");
    return 2;
  }
  const char *seeds = opt(argc, argv, "--seeds", "0..15");
  unsigned long a = 0, b = 15;
  std::sscanf(seeds, "%lu..%lu", &a, &b);
  const double half = std::atof(opt(argc, argv, "--half", "120"));
  const double cell = std::atof(opt(argc, argv, "--cell", "0.5"));
  const std::string preset = opt(argc, argv, "--preset", "standard");
  const std::string ontology = opt(argc, argv, "--ontology", "merged_default");
  int made = 0;
  for (unsigned long s = a; s <= b; ++s) {
    world_params wp;
    wp.min_x = wp.min_y = -half;
    wp.max_x = wp.max_y = half;
    wp.seed = s;
    wp.ontology = ontology;
    apply_preset(wp, preset);
    world_model wm = world_model::generate(wp);
    grid_spec g = grid_spec::window(0, 0, half, cell);
    raster_out ro;
    raster(wm, g, ro);
    bundle_options bo;
    bo.previews = !flag(argc, argv, "--no-preview");
    char dir[512];
    std::snprintf(dir, sizeof(dir), "%s/w%04lu", out, s);
    write_bundle(dir, wm, g, ro, bo);
    ++made;
  }
  std::printf("wrote %d bundles to %s\n", made, out);
  return 0;
}

int cmd_inspect(int argc, char **argv) {
  if (argc < 3)
    return usage();
  std::ifstream f(std::string(argv[2]) + "/manifest.json", std::ios::binary);
  if (!f) {
    std::fprintf(stderr, "no manifest.json in %s\n", argv[2]);
    return 1;
  }
  std::ostringstream ss;
  ss << f.rdbuf();
  std::printf("%s\n", ss.str().c_str());
  return 0;
}

} // namespace

int main(int argc, char **argv) try {
  if (argc < 2)
    return usage();
  const std::string cmd = argv[1];
  if (cmd == "build")
    return cmd_build(argc, argv);
  if (cmd == "sample")
    return cmd_sample(argc, argv);
  if (cmd == "inspect")
    return cmd_inspect(argc, argv);
  return usage();
} catch (const std::exception &e) {
  std::fprintf(stderr, "error: %s\n", e.what());
  return 1;
}
