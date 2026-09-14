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

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/image/image.h>
#include <cvc/world/bundle.h>
#include <cvc/world/npy.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace cvc {
namespace world {

namespace {

double sigma_m_for(const std::string &kind) {
  // outdoor: ~2.0 m (the tuned 2.1 m frame). indoor/mixed: 0.5 m (= 1 cell at
  // 0.5 m/cell), because a 2 m blur across a 1.5 m corridor destroys it (§7.1a).
  return kind == "outdoor" ? 2.0 : 0.5;
}

void write_text(const std::string &path, const std::string &s) {
  std::ofstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("cannot write " + path);
  f << s;
}

// A binary PPM (P6) fallback when no PNG handler is registered.
void write_ppm(const std::string &path, const std::vector<std::uint8_t> &rgb, int w, int h) {
  std::ofstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("cannot write " + path);
  f << "P6\n" << w << " " << h << "\n255\n";
  f.write(reinterpret_cast<const char *>(rgb.data()), std::streamsize(rgb.size()));
}

void write_preview(const std::string &png_path, const std::vector<std::uint8_t> &rgb, int w,
                   int h) {
  try {
    cvc::image img(w, h, cvc::image::pixel_format::RGB, cvc::image::data_type::u8, rgb.data());
    cvc::write_image(img, png_path);
  } catch (...) {
    // Fall back to a .ppm next to the requested path.
    std::string ppm = png_path;
    auto dot = ppm.rfind('.');
    if (dot != std::string::npos)
      ppm = ppm.substr(0, dot);
    write_ppm(ppm + ".ppm", rgb, w, h);
  }
}

} // namespace

std::string bundle_manifest(const world_model &wm, const grid_spec &g, const raster_out &out,
                            const bundle_options &opt) {
  const std::size_t n = out.klass.size();
  const surface_registry &reg = wm.registry();

  // Statistics.
  std::map<std::uint16_t, std::size_t> class_hist;
  double rsum = 0, rsum2 = 0, rmax = 0;
  std::size_t hard = 0, occ = 0;
  for (std::size_t i = 0; i < n; ++i) {
    ++class_hist[out.klass[i]];
    double rr = out.risk_raw[i];
    rsum += rr;
    rsum2 += rr * rr;
    rmax = std::max(rmax, rr);
    hard += out.hard[i] ? 1 : 0;
    occ += out.occupancy[i] ? 1 : 0;
  }
  const double inv = n ? 1.0 / double(n) : 0.0;
  const double rmean = rsum * inv;
  const double rstd = std::sqrt(std::max(0.0, rsum2 * inv - rmean * rmean));

  const double cell_w = g.cell_w();
  const double sigma_m = sigma_m_for(opt.scene_kind);
  const double sigma_cells = cell_w > 0 ? sigma_m / cell_w : 0.0;

  std::ostringstream o;
  char b[512];
  o << "{\n";
  o << "  \"format\": \"cvcworld/2\",\n";
  o << "  \"tool\": {\"name\": \"cvc-worldgen\", \"version\": \"" << opt.tool_version
    << "\", \"libcvc\": \"" << opt.libcvc_commit << "\"},\n";
  o << "  \"scene_kind\": \"" << opt.scene_kind << "\",\n";
  std::snprintf(b, sizeof(b),
                "  \"grid\": {\"rows\": %d, \"cols\": %d, "
                "\"bounds\": [%.17g, %.17g, %.17g, %.17g], "
                "\"cell_w\": %.17g, \"cell_h\": %.17g, "
                "\"cell_w_formula\": \"(max_x - min_x) / (cols - 1)\", \"row_order\": \"%s\"},\n",
                g.rows, g.cols, g.min_x, g.min_y, g.max_x, g.max_y, cell_w, g.cell_h(),
                grid_spec::row_order);
  o << b;
  std::snprintf(b, sizeof(b),
                "  \"frame\": {\"scale\": %.6g, \"agent_radius_m\": %.6g, \"sigma_m\": %.6g, "
                "\"sigma_recommended_cells\": %.6g, \"blur_bleed_radius_m\": %.6g, "
                "\"lam_soft_scale_hint\": %.6g, \"gate_horizon_recommended_cells\": %d, "
                "\"note\": \"sigma is recorded as a LENGTH; the consumer measures it in cells "
                "(roadmap 7.1a).\"},\n",
                opt.scale, opt.agent_radius_m, sigma_m, sigma_cells, 4.0 * sigma_m, sigma_m / 2.1,
                int(std::lround(cell_w > 0 ? 25.0 / cell_w : 0.0)));
  o << b;
  std::snprintf(b, sizeof(b),
                "  \"material\": {\"ontology\": \"%s\", \"ontology_hash\": \"b3:%016llx\", "
                "\"hard_class_rho\": 0.0, \"note\": \"risk_raw and hard are RAW contract inputs; "
                "all derived planes belong to cvc::nav::material_build.\"},\n",
                reg.ontology().c_str(), (unsigned long long)reg.ontology_hash());
  o << b;
  // Top class fractions.
  o << "  \"stats\": {\"class_fractions\": {";
  {
    std::vector<std::pair<std::uint16_t, std::size_t>> v(class_hist.begin(), class_hist.end());
    // TOTAL order (count desc, then class id asc) so the JSON order — and which
    // classes survive the top-8 cutoff — is identical across STL implementations.
    // A count-only comparator is not a strict weak ordering for ties and diverges
    // between libstdc++/libc++, breaking manifest reproducibility.
    std::sort(v.begin(), v.end(), [](const auto &a, const auto &c) {
      return a.second != c.second ? a.second > c.second : a.first < c.first;
    });
    int emitted = 0;
    for (auto &kv : v) {
      if (emitted >= 8)
        break;
      std::snprintf(b, sizeof(b), "%s\"%s\": %.4f", emitted ? ", " : "", reg[kv.first].name,
                    double(kv.second) * inv);
      o << b;
      ++emitted;
    }
  }
  std::snprintf(b, sizeof(b),
                "}, \"risk_mean\": %.4f, \"risk_std\": %.4f, \"risk_max\": %.4f, "
                "\"hard_fraction\": %.5f, \"occupancy_fraction\": %.5f},\n",
                rmean, rstd, rmax, double(hard) * inv, double(occ) * inv);
  o << b;
  std::snprintf(b, sizeof(b), "  \"seed\": %llu\n}\n", (unsigned long long)wm.params().seed);
  o << b;
  return o.str();
}

std::string write_bundle(const std::string &dir, const world_model &wm, const grid_spec &g,
                         const raster_out &out, const bundle_options &opt) {
  namespace fs = std::filesystem;
  fs::create_directories(fs::path(dir) / "layer00");
  const std::string L = (fs::path(dir) / "layer00").string() + "/";

  write_npy(L + "class.npy", out.klass, out.rows, out.cols);
  write_npy(L + "risk_raw.npy", out.risk_raw, out.rows, out.cols);
  write_npy(L + "hard.npy", out.hard, out.rows, out.cols);
  write_npy(L + "occupancy.npy", out.occupancy, out.rows, out.cols);
  write_npy(L + "height.npy", out.height, out.rows, out.cols);
  write_npy(L + "layer_owner.npy", out.layer_owner, out.rows, out.cols);

  const surface_registry &reg = wm.registry();
  write_text((fs::path(dir) / "registry.json").string(), reg.to_json());

  const std::string manifest = bundle_manifest(wm, g, out, opt);
  write_text((fs::path(dir) / "manifest.json").string(), manifest);

  // Provenance + regeneration params.
  {
    char b[512];
    std::snprintf(
        b, sizeof(b),
        "{\n  \"seed\": %llu,\n  \"ontology\": \"%s\",\n  \"ontology_hash\": "
        "\"b3:%016llx\",\n  \"bounds\": [%.17g, %.17g, %.17g, %.17g],\n  \"tool_version\": "
        "\"%s\",\n  \"libcvc\": \"%s\"\n}\n",
        (unsigned long long)wm.params().seed, reg.ontology().c_str(),
        (unsigned long long)reg.ontology_hash(), wm.params().min_x, wm.params().min_y,
        wm.params().max_x, wm.params().max_y, opt.tool_version.c_str(), opt.libcvc_commit.c_str());
    write_text((fs::path(dir) / "provenance.json").string(), b);
  }

  if (opt.previews) {
    const int w = out.cols, h = out.rows;
    std::vector<std::uint8_t> cls(std::size_t(w) * h * 3), rsk(std::size_t(w) * h * 3),
        hrd(std::size_t(w) * h * 3);
    for (std::size_t i = 0; i < out.klass.size(); ++i) {
      const surface_class &sc = reg[out.klass[i]];
      cls[i * 3 + 0] = std::uint8_t(sc.albedo[0] * 255);
      cls[i * 3 + 1] = std::uint8_t(sc.albedo[1] * 255);
      cls[i * 3 + 2] = std::uint8_t(sc.albedo[2] * 255);
      std::uint8_t rv = std::uint8_t(std::min(1.0f, out.risk_raw[i]) * 255);
      rsk[i * 3 + 0] = rsk[i * 3 + 1] = rsk[i * 3 + 2] = rv;
      std::uint8_t hv = out.hard[i] ? 255 : 0;
      hrd[i * 3 + 0] = hrd[i * 3 + 1] = hrd[i * 3 + 2] = hv;
    }
    write_preview((fs::path(dir) / "class_preview.png").string(), cls, w, h);
    write_preview((fs::path(dir) / "risk_preview.png").string(), rsk, w, h);
    write_preview((fs::path(dir) / "hard_preview.png").string(), hrd, w, h);
  }

  return manifest;
}

} // namespace world
} // namespace cvc
