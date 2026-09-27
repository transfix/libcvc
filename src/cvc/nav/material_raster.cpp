/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// material_raster.cpp — see material_raster.h. Pure-C++ twin of grl_snam.tools.material_raster:
// SAME classification + SAME cvc-scene-material/1 output. Prefers the bundle's authoritative
// land-cover masks (foliage_mask.png land classes + roads/water/open_fields alpha masks) and falls
// back to a deterministic color classification of satellite.png. All output grids are oriented row
// 0
// == world min_y (the sim_world/nav_samplers occupancy convention), so the north-up source imagery
// is flipped in y.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cvc/image/image.h>
#include <cvc/nav/material_raster.h>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>

namespace cvc {
namespace nav {

float material_mu(int id) {
  switch (id) {
  case MAT_FOLIAGE:
    return 0.65f;
  case MAT_SOIL:
    return 0.55f;
  case MAT_ROCK:
    return 0.50f;
  case MAT_WATER:
    return 0.30f;
  default:
    return 1.0f; // open_air + building materials
  }
}

float material_risk(int id) {
  switch (id) {
  case MAT_FOLIAGE:
    return 0.30f;
  case MAT_SOIL:
    return 0.50f;
  case MAT_ROCK:
    return 0.80f;
  case MAT_WATER:
    return 1.0f;
  default:
    return 0.0f;
  }
}

const char *material_name(int id) {
  static const char *kNames[kMaterialCount] = {"reinforced_concrete",
                                               "brick",
                                               "glass",
                                               "wood",
                                               "foliage",
                                               "drywall",
                                               "metal",
                                               "open_air",
                                               "soil",
                                               "water",
                                               "glass_laminated",
                                               "composite_panel",
                                               "rock"};
  return (id >= 0 && id < kMaterialCount) ? kNames[id] : "";
}

namespace {

// Mean-pool a per-pixel classified grid (image orientation) to (rows, cols) by MAJORITY class and
// flip in y so out row 0 == world min_y. `pix` is [H*W] material ids.
material_raster majority_downsample_flip(const std::vector<int> &pix, int W, int H, int rows,
                                         int cols) {
  const int ph = H / rows, pw = W / cols;
  if (ph < 1 || pw < 1)
    throw std::runtime_error("cvc::nav::material_raster: source smaller than the grid");
  material_raster out;
  out.rows = rows;
  out.cols = cols;
  out.material_id.assign(static_cast<std::size_t>(rows) * cols, MAT_OPEN_AIR);
  std::vector<int> hist(kMaterialCount, 0);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      std::fill(hist.begin(), hist.end(), 0);
      for (int yy = r * ph; yy < (r + 1) * ph; ++yy)
        for (int xx = c * pw; xx < (c + 1) * pw; ++xx) {
          const int m = pix[static_cast<std::size_t>(yy) * W + xx];
          if (m >= 0 && m < kMaterialCount)
            ++hist[m];
        }
      // Majority class, tie -> earlier in this order (matches grl_snam: open_air, soil, foliage,
      // water, rock — the only ids the classifier emits), so C++ and Python pick identically.
      static const int kOrder[] = {MAT_OPEN_AIR, MAT_SOIL, MAT_FOLIAGE, MAT_WATER, MAT_ROCK};
      int best = MAT_OPEN_AIR, bestn = -1;
      for (int m : kOrder)
        if (hist[m] > bestn) {
          bestn = hist[m];
          best = m;
        }
      // flip y: image row r (top == max_y) -> grid row (rows-1-r) so grid row 0 == min_y.
      out.material_id[static_cast<std::size_t>(rows - 1 - r) * cols + c] = best;
    }
  }
  return out;
}

// Load a PNG (RGBA or grayscale) into an interleaved uint8 buffer; returns false if absent.
bool load_image(const std::string &path, std::vector<std::uint8_t> &buf, int &w, int &h, int &ch) {
  namespace fs = std::filesystem;
  std::error_code ec;
  if (!fs::exists(path, ec))
    return false;
  image im = image::load(path);
  if (im.empty())
    return false;
  w = im.width();
  h = im.height();
  ch = im.channels();
  buf.assign(im.data(), im.data() + static_cast<std::size_t>(w) * h * ch);
  return true;
}

// foliage_mask.png land class (unet) -> palette id; 0 = none.
int foliage_class(int v) {
  switch (v) {
  case 1: // tree
  case 2: // grass
  case 4: // shrub
    return MAT_FOLIAGE;
  case 3:
    return MAT_ROCK;
  case 5:
    return MAT_SOIL;
  default:
    return -1;
  }
}

} // namespace

material_raster classify_satellite(const std::uint8_t *rgb, int W, int H, int ch, int rows,
                                   int cols, double min_x, double min_y, double max_x,
                                   double max_y) {
  const int ph = H / rows, pw = W / cols;
  if (ph < 1 || pw < 1)
    throw std::runtime_error("cvc::nav::classify_satellite: image smaller than the grid");
  std::vector<int> pix(static_cast<std::size_t>(H) * W, MAT_OPEN_AIR);
  for (int yy = 0; yy < H; ++yy)
    for (int xx = 0; xx < W; ++xx) {
      const std::uint8_t *p = rgb + (static_cast<std::size_t>(yy) * W + xx) * ch;
      const double r = p[0], g = (ch >= 2 ? p[1] : p[0]), b = (ch >= 3 ? p[2] : p[0]);
      const double bright = (r + g + b) / 3.0, exg = 2.0 * g - r - b, warm = r - b;
      int m = MAT_OPEN_AIR;
      if (warm > 14 && r > g && g >= b && exg < 6)
        m = MAT_SOIL;
      if (exg > 14 && g >= b)
        m = MAT_FOLIAGE;
      if (bright > 170 && warm > 6 && exg < 8 && r > g)
        m = MAT_ROCK;
      if (b > r + 6 && b > g && bright < 125)
        m = MAT_WATER;
      pix[static_cast<std::size_t>(yy) * W + xx] = m;
    }
  material_raster out = majority_downsample_flip(pix, W, H, rows, cols);
  out.min_x = min_x;
  out.min_y = min_y;
  out.max_x = max_x;
  out.max_y = max_y;
  return out;
}

namespace {

// Build the per-pixel material from the land-cover masks (returns false if foliage_mask is absent).
bool masks_pixels(const std::string &dir, std::vector<int> &pix, int &W, int &H) {
  std::vector<std::uint8_t> fol;
  int fw = 0, fh = 0, fc = 0;
  if (!load_image(dir + "/foliage_mask.png", fol, fw, fh, fc))
    return false;
  W = fw;
  H = fh;
  pix.assign(static_cast<std::size_t>(H) * W, MAT_OPEN_AIR);

  auto apply_alpha = [&](const std::string &name, int mat) {
    std::vector<std::uint8_t> im;
    int w = 0, h = 0, c = 0;
    if (!load_image(dir + "/" + name, im, w, h, c) || w != W || h != H)
      return;
    for (std::size_t i = 0; i < static_cast<std::size_t>(W) * H; ++i) {
      const bool on = (c == 4) ? im[i * c + 3] > 0 : im[i * c] > 0;
      if (on)
        pix[i] = mat;
    }
  };

  // priority, later wins: open_air default -> open_fields=soil -> foliage classes -> water ->
  // roads.
  apply_alpha("open_fields.png", MAT_SOIL);
  for (std::size_t i = 0; i < static_cast<std::size_t>(W) * H; ++i) {
    const int fm = foliage_class(fol[i * fc]);
    if (fm >= 0)
      pix[i] = fm;
  }
  apply_alpha("water.png", MAT_WATER);
  apply_alpha("roads.png", MAT_OPEN_AIR);
  return true;
}

// Parse rows/cols/bounds from a bundle's terrain.json (hand-rolled, no JSON dep — like nav_common).
bool parse_terrain(const std::string &path, int &rows, int &cols, double &min_x, double &min_y,
                   double &max_x, double &max_y) {
  std::ifstream f(path);
  if (!f)
    return false;
  const std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  auto num = [&](const char *k, double &v) -> bool {
    const auto p = s.find(k);
    if (p == std::string::npos)
      return false;
    const auto c = s.find(':', p);
    if (c == std::string::npos)
      return false;
    v = std::atof(s.c_str() + c + 1);
    return true;
  };
  double rr = 0, cc = 0;
  return num("\"rows\"", rr) && num("\"cols\"", cc) && num("\"min_x\"", min_x) &&
         num("\"min_y\"", min_y) && num("\"max_x\"", max_x) && num("\"max_y\"", max_y) &&
         (rows = static_cast<int>(rr)) > 1 && (cols = static_cast<int>(cc)) > 1;
}

} // namespace

material_raster segment_scene_material(const std::string &bundle_dir) {
  int rows = 0, cols = 0;
  double min_x = 0, min_y = 0, max_x = 0, max_y = 0;
  if (!parse_terrain(bundle_dir + "/terrain.json", rows, cols, min_x, min_y, max_x, max_y))
    throw std::runtime_error("cvc::nav::segment_scene_material: bad/missing terrain.json in " +
                             bundle_dir);

  // Prefer the authoritative land-cover masks.
  std::vector<int> pix;
  int W = 0, H = 0;
  if (masks_pixels(bundle_dir, pix, W, H)) {
    material_raster out = majority_downsample_flip(pix, W, H, rows, cols);
    out.min_x = min_x;
    out.min_y = min_y;
    out.max_x = max_x;
    out.max_y = max_y;
    return out;
  }

  // Fall back to classifying satellite.png (the lean-bundle path).
  std::vector<std::uint8_t> sat;
  int sw = 0, sh = 0, sc = 0;
  if (!load_image(bundle_dir + "/satellite.png", sat, sw, sh, sc))
    throw std::runtime_error(
        "cvc::nav::segment_scene_material: no land-cover masks and no satellite.png in " +
        bundle_dir);
  return classify_satellite(sat.data(), sw, sh, sc, rows, cols, min_x, min_y, max_x, max_y);
}

std::string material_raster::to_json() const {
  std::ostringstream o;
  auto fnum = [&](double v) {
    std::ostringstream t;
    t << v; // shortest reasonable form (parity is on the id grid, not the float text)
    return t.str();
  };
  o << "{\"schema\":\"cvc-scene-material/1\",\"provenance\":\"scene land cover (cvc::nav "
       "material_raster: masks if present, else satellite); ids = cvc::dbg MATERIAL_TABLE\",";
  o << "\"rows\":" << rows << ",\"cols\":" << cols << ",\"bounds\":{\"min_x\":" << fnum(min_x)
    << ",\"min_y\":" << fnum(min_y) << ",\"max_x\":" << fnum(max_x) << ",\"max_y\":" << fnum(max_y)
    << "},";
  o << "\"palette\":[";
  for (int m = 0; m < kMaterialCount; ++m)
    o << (m ? "," : "") << "\"" << material_name(m) << "\"";
  o << "],";
  // mu/risk tables over the ids actually present (sorted), matching the Python.
  std::vector<int> present;
  for (int m = 0; m < kMaterialCount; ++m)
    if (std::find(material_id.begin(), material_id.end(), m) != material_id.end())
      present.push_back(m);
  auto table = [&](const char *key, float (*fn)(int)) {
    o << "\"" << key << "\":{";
    for (std::size_t i = 0; i < present.size(); ++i)
      o << (i ? "," : "") << "\"" << present[i] << "\":" << fnum(fn(present[i]));
    o << "},";
  };
  table("mu", material_mu);
  table("risk", material_risk);
  o << "\"material_id\":[";
  for (int r = 0; r < rows; ++r) {
    o << (r ? ",[" : "[");
    for (int c = 0; c < cols; ++c)
      o << (c ? "," : "") << material_id[static_cast<std::size_t>(r) * cols + c];
    o << "]";
  }
  o << "]}";
  return o.str();
}

bool material_raster::write_json(const std::string &path) const {
  std::ofstream f(path);
  if (!f)
    return false;
  f << to_json();
  return static_cast<bool>(f);
}

} // namespace nav
} // namespace cvc
