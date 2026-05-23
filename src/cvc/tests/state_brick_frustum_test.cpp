/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_brick_manifest.h>
#include <gtest/gtest.h>

using cvc::brick_extent;
using cvc::state_brick_manifest;

class StateBrickFrustumTest : public ::testing::Test {
protected:
  state_brick_manifest manifest;

  void SetUp() override {
    // Create a 3x3x1 grid of bricks each 10x10x10 voxels.
    for (int x = 0; x < 3; ++x) {
      for (int y = 0; y < 3; ++y) {
        brick_extent e;
        e.origin_x = x * 10;
        e.origin_y = y * 10;
        e.origin_z = 0;
        e.size_x = 10;
        e.size_y = 10;
        e.size_z = 10;
        manifest.chunks.push_back("chunk_" + std::to_string(x) + "_" + std::to_string(y));
        manifest.chunk_bytes.push_back(1000);
        manifest.extents.push_back(e);
      }
    }
  }
};

TEST_F(StateBrickFrustumTest, AllPlanesPassingReturnsAll) {
  // Planes that encompass everything: each plane's positive half-space
  // includes the origin and extends far. Use a giant box frustum.
  state_brick_manifest::plane planes[6] = {
      {1, 0, 0, 100},  // x >= -100
      {-1, 0, 0, 100}, // -x >= -100 i.e. x <= 100
      {0, 1, 0, 100},  // y >= -100
      {0, -1, 0, 100}, // y <= 100
      {0, 0, 1, 100},  // z >= -100
      {0, 0, -1, 100}, // z <= 100
  };
  auto result = manifest.bricks_in_frustum(planes);
  EXPECT_EQ(result.size(), 9u);
}

TEST_F(StateBrickFrustumTest, TightFrustumSelectsSubset) {
  // Planes that only include the first brick (0..10, 0..10, 0..10).
  state_brick_manifest::plane planes[6] = {
      {1, 0, 0, 0},   // x >= 0
      {-1, 0, 0, 10}, // x <= 10
      {0, 1, 0, 0},   // y >= 0
      {0, -1, 0, 10}, // y <= 10
      {0, 0, 1, 0},   // z >= 0
      {0, 0, -1, 10}, // z <= 10
  };
  auto result = manifest.bricks_in_frustum(planes);
  // At least the first brick should be included.
  EXPECT_GE(result.size(), 1u);
  EXPECT_LE(result.size(), 9u);
  // The result should include brick index 0.
  EXPECT_NE(std::find(result.begin(), result.end(), 0u), result.end());
}

TEST_F(StateBrickFrustumTest, FrustumFarAwayReturnsEmpty) {
  // Planes that are far from any brick (everything at x > 1000).
  state_brick_manifest::plane planes[6] = {
      {1, 0, 0, -1000}, // x >= 1000
      {-1, 0, 0, 2000}, // x <= 2000
      {0, 1, 0, -1000}, // y >= 1000
      {0, -1, 0, 2000}, // y <= 2000
      {0, 0, 1, -1000}, // z >= 1000
      {0, 0, -1, 2000}, // z <= 2000
  };
  auto result = manifest.bricks_in_frustum(planes);
  EXPECT_EQ(result.size(), 0u);
}

TEST_F(StateBrickFrustumTest, EmptyManifestReturnsEmpty) {
  state_brick_manifest empty;
  state_brick_manifest::plane planes[6] = {
      {1, 0, 0, 100},  {-1, 0, 0, 100}, {0, 1, 0, 100},
      {0, -1, 0, 100}, {0, 0, 1, 100},  {0, 0, -1, 100},
  };
  auto result = empty.bricks_in_frustum(planes);
  EXPECT_EQ(result.size(), 0u);
}

TEST_F(StateBrickFrustumTest, RegionQueryBaseline) {
  // Also test bricks_in_region as a baseline sanity check.
  auto result = manifest.bricks_in_region(0, 0, 0, 30, 30, 10);
  EXPECT_EQ(result.size(), 9u);

  auto subset = manifest.bricks_in_region(0, 0, 0, 10, 10, 10);
  EXPECT_EQ(subset.size(), 1u);
}
