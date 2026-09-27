// cvc::nav material_raster — the pure-C++ satellite/mask -> material palette segmenter. Mirrors the
// grl_snam.tools.material_raster tests (same classifier, same mask priority, same orientation).

#include <cstdint>
#include <cvc/image/image.h>
#include <cvc/nav/material_raster.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace cvc::nav;
namespace fs = std::filesystem;

TEST(NavMaterialRaster, PaletteGripAndRisk) {
  EXPECT_FLOAT_EQ(material_mu(MAT_OPEN_AIR), 1.0f);
  EXPECT_FLOAT_EQ(material_mu(MAT_FOLIAGE), 0.65f);
  EXPECT_FLOAT_EQ(material_mu(MAT_WATER), 0.30f);
  EXPECT_FLOAT_EQ(material_mu(MAT_REINFORCED_CONCRETE), 1.0f); // building material -> full grip
  EXPECT_FLOAT_EQ(material_risk(MAT_WATER), 1.0f);
  EXPECT_FLOAT_EQ(material_risk(MAT_OPEN_AIR), 0.0f);
  EXPECT_STREQ(material_name(MAT_FOLIAGE), "foliage");
  EXPECT_STREQ(material_name(MAT_OPEN_AIR), "open_air");
}

TEST(NavMaterialRaster, ClassifySatelliteTagsAndFlipsToSimOrientation) {
  // 2x2 north-up RGB (1px/cell): top [green, gray], bottom [tan, blue].
  const std::uint8_t rgb[2 * 2 * 3] = {
      50,  160, 50, 150, 150, 150, // north row
      175, 130, 90, 40,  90,  200, // south row
  };
  material_raster m = classify_satellite(rgb, 2, 2, 3, 2, 2, -100, -50, 100, 50);
  ASSERT_EQ(m.rows, 2);
  ASSERT_EQ(m.cols, 2);
  // flipped so row 0 == world min_y (image bottom): [tan->soil, blue->water]
  EXPECT_EQ(m.at(0, 0), MAT_SOIL);
  EXPECT_EQ(m.at(0, 1), MAT_WATER);
  // row 1 == image top: [green->foliage, gray->open_air]
  EXPECT_EQ(m.at(1, 0), MAT_FOLIAGE);
  EXPECT_EQ(m.at(1, 1), MAT_OPEN_AIR);
  EXPECT_DOUBLE_EQ(m.min_y, -50);
}

static void save_gray(const fs::path &p, int w, int h, const std::vector<std::uint8_t> &px) {
  cvc::image im(w, h, cvc::image::pixel_format::GRAY, cvc::image::data_type::u8, px.data());
  im.save(p.string());
}
static void save_rgba(const fs::path &p, int w, int h, const std::vector<std::uint8_t> &px) {
  cvc::image im(w, h, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8, px.data());
  im.save(p.string());
}

TEST(NavMaterialRaster, SegmentPrefersMasksWithPriorityAndFlip) {
  const fs::path dir = fs::temp_directory_path() / "cvc_matraster_masks_test";
  fs::create_directories(dir);
  std::ofstream(dir / "terrain.json")
      << R"({"rows":2,"cols":2,"bounds":{"min_x":-8,"min_y":-8,"max_x":8,"max_y":8}})";
  // foliage_mask (GRAY, land classes): top [tree(1), none(0)], bottom [bare(5), none(0)]
  save_gray(dir / "foliage_mask.png", 2, 2, {1, 0, 5, 0});
  // water over image (0,1); roads over image (1,1)  (alpha 255 where masked)
  save_rgba(dir / "water.png", 2, 2, {0, 0, 0, 0, 10, 10, 10, 255, 0, 0, 0, 0, 0, 0, 0, 0});
  save_rgba(dir / "roads.png", 2, 2, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 10, 10, 255});

  material_raster m = segment_scene_material(dir.string());
  ASSERT_EQ(m.rows, 2);
  ASSERT_EQ(m.cols, 2);
  // flipped: row 0 == min_y == image bottom row -> [bare->soil, roads->open_air]
  EXPECT_EQ(m.at(0, 0), MAT_SOIL);
  EXPECT_EQ(m.at(0, 1), MAT_OPEN_AIR);
  // row 1 == image top -> [tree->foliage, water->water]
  EXPECT_EQ(m.at(1, 0), MAT_FOLIAGE);
  EXPECT_EQ(m.at(1, 1), MAT_WATER);

  // to_json carries the schema, the flipped id grid, and palette-sourced mu/risk.
  const std::string j = m.to_json();
  EXPECT_NE(j.find("\"schema\":\"cvc-scene-material/1\""), std::string::npos);
  EXPECT_NE(j.find("\"material_id\":[[8,7],[4,9]]"),
            std::string::npos);                    // soil,open_air / foliage,water
  EXPECT_NE(j.find("\"9\":1"), std::string::npos); // water risk 1.0 present
  fs::remove_all(dir);
}

TEST(NavMaterialRaster, SegmentFallsBackToSatelliteWithoutMasks) {
  const fs::path dir = fs::temp_directory_path() / "cvc_matraster_sat_test";
  fs::create_directories(dir);
  std::ofstream(dir / "terrain.json")
      << R"({"rows":2,"cols":2,"bounds":{"min_x":-8,"min_y":-8,"max_x":8,"max_y":8}})";
  // no masks -> classify satellite.png. All gray -> open_air.
  save_rgba(dir / "satellite.png", 2, 2,
            {150, 150, 150, 255, 150, 150, 150, 255, 150, 150, 150, 255, 150, 150, 150, 255});
  material_raster m = segment_scene_material(dir.string());
  for (int i = 0; i < m.rows * m.cols; ++i)
    EXPECT_EQ(m.material_id[i], MAT_OPEN_AIR);
  fs::remove_all(dir);
}
