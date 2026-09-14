/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/world/bundle.h for the full header).
*/

// The bundle is the file seam to the consumer. The manifest must carry the exact
// grid facts a Python (bounds+center) and a C++ (rows,cols,cell_w) consumer each
// need, with cell_w == (max_x-min_x)/(cols-1) so the frame can never be guessed
// wrong (roadmap §7.1/§7.1a), and row_order pinned to min_y_first (§7.6).

#include <cstdint>
#include <cstdio>
#include <cvc/world/bundle.h>
#include <cvc/world/npy.h>
#include <cvc/world/raster.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>
#include <string>

using namespace cvc::world;

namespace {
world_model make_world() {
  world_params wp;
  wp.seed = 1;
  wp.min_x = wp.min_y = -64;
  wp.max_x = wp.max_y = 64;
  return world_model::generate(wp);
}
std::string read_file(const std::string &p) {
  std::ifstream f(p, std::ios::binary);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}
} // namespace

TEST(WorldBundle, NpyHeaderAndShape) {
  std::vector<std::uint16_t> data(6 * 4, 7);
  auto b = npy_bytes(data, 6, 4);
  ASSERT_GE(b.size(), std::size_t(80));
  EXPECT_EQ(b[0], 0x93);
  EXPECT_EQ(b[1], 'N');
  EXPECT_EQ((b.size() - 10 - (b[8] | (b[9] << 8))), data.size() * 2); // payload bytes
  std::string hdr(b.begin() + 10, b.begin() + 10 + (b[8] | (b[9] << 8)));
  EXPECT_NE(hdr.find("'<u2'"), std::string::npos);
  EXPECT_NE(hdr.find("(6, 4)"), std::string::npos);
  EXPECT_NE(hdr.find("'fortran_order': False"), std::string::npos);
}

TEST(WorldBundle, ManifestCarriesConsumerFrameFacts) {
  world_model wm = make_world();
  grid_spec g = grid_spec::window(0, 0, 64, 0.5); // 257x257, cell 0.5
  raster_out o;
  raster(wm, g, o);
  std::string m = bundle_manifest(wm, g, o, bundle_options{});

  EXPECT_NE(m.find("\"row_order\": \"min_y_first\""), std::string::npos);
  EXPECT_NE(m.find("cvcworld/2"), std::string::npos);
  EXPECT_NE(m.find("\"ontology_hash\""), std::string::npos);
  EXPECT_NE(m.find("sigma_m"), std::string::npos);
  // cell_w in the manifest must be the consumer's formula, bit-for-bit.
  const double expect = (g.max_x - g.min_x) / double(g.cols - 1);
  EXPECT_DOUBLE_EQ(expect, 0.5);
  char needle[64];
  std::snprintf(needle, sizeof(needle), "\"cell_w\": %.10g", expect);
  EXPECT_NE(m.find(needle), std::string::npos) << m;
}

TEST(WorldBundle, WritesLoadableFiles) {
  world_model wm = make_world();
  grid_spec g = grid_spec::window(0, 0, 64, 2.0); // small for speed
  raster_out o;
  raster(wm, g, o);
  namespace fs = std::filesystem;
  fs::path dir = fs::temp_directory_path() / "cvcworld_bundle_test";
  fs::remove_all(dir);
  bundle_options bo;
  bo.previews = false; // avoid needing a PNG handler in the test
  write_bundle(dir.string(), wm, g, o, bo);

  EXPECT_TRUE(fs::exists(dir / "manifest.json"));
  EXPECT_TRUE(fs::exists(dir / "registry.json"));
  EXPECT_TRUE(fs::exists(dir / "provenance.json"));
  EXPECT_TRUE(fs::exists(dir / "layer00" / "class.npy"));
  EXPECT_TRUE(fs::exists(dir / "layer00" / "risk_raw.npy"));
  EXPECT_TRUE(fs::exists(dir / "layer00" / "hard.npy"));
  EXPECT_TRUE(fs::exists(dir / "layer00" / "occupancy.npy"));

  // class.npy payload size == rows*cols*2 + header.
  std::string cls = read_file((dir / "layer00" / "class.npy").string());
  std::size_t hlen = (std::uint8_t)cls[8] | ((std::uint8_t)cls[9] << 8);
  EXPECT_EQ(cls.size(), 10 + hlen + std::size_t(g.rows) * g.cols * 2);
  fs::remove_all(dir);
}

TEST(WorldBundle, PreviewsWriteAnImageOrPpmFallback) {
  world_model wm = make_world();
  grid_spec g = grid_spec::window(0, 0, 64, 2.0);
  raster_out o;
  raster(wm, g, o);
  namespace fs = std::filesystem;
  fs::path dir = fs::temp_directory_path() / "cvcworld_preview_test";
  fs::remove_all(dir);
  bundle_options bo;
  bo.previews = true; // exercise the preview path (PNG, or PPM fallback)
  write_bundle(dir.string(), wm, g, o, bo);
  // Either a PNG handler produced class_preview.png, or the fallback wrote .ppm.
  EXPECT_TRUE(fs::exists(dir / "class_preview.png") || fs::exists(dir / "class_preview.ppm"));
  EXPECT_TRUE(fs::exists(dir / "risk_preview.png") || fs::exists(dir / "risk_preview.ppm"));
  fs::remove_all(dir);
}

TEST(WorldBundle, DeterministicBytes) {
  world_model wm = make_world();
  grid_spec g = grid_spec::window(0, 0, 64, 2.0);
  raster_out o;
  raster(wm, g, o);
  EXPECT_EQ(npy_bytes(o.klass, o.rows, o.cols), npy_bytes(o.klass, o.rows, o.cols));
}
