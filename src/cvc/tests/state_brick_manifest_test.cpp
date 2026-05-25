/*
  Copyright 2026 The University of Texas at Austin
  Tests for brick-aware manifests, brick writer/reader, and spatial queries.
*/

#include <cstring>
#include <cvc/state_blob_store.h>
#include <cvc/state_brick_manifest.h>
#include <cvc/state_compression_registry.h>
#include <cvc/types.h>
#include <gtest/gtest.h>

using namespace cvc;

class BrickManifestTest : public ::testing::Test {
protected:
  memory_state_blob_store store;
  state_compression_registry compression;
};

// ---- manifest serialization ----

TEST_F(BrickManifestTest, SerializeParseRoundTrip) {
  state_brick_manifest m;
  m.version = 1;
  m.chunk_size = 32;
  m.total_size = 1024;
  m.codec = "raw";
  m.content_digest = "abc123";
  m.has_volume_header = true;
  m.vol_xdim = 64;
  m.vol_ydim = 64;
  m.vol_zdim = 64;
  m.voxel_type = static_cast<std::uint32_t>(Float);
  m.brick_xdim = 32;
  m.brick_ydim = 32;
  m.brick_zdim = 32;

  m.chunks = {"chunk1", "chunk2"};
  m.chunk_bytes = {512, 512};

  brick_extent e1;
  e1.origin_x = 0;
  e1.origin_y = 0;
  e1.origin_z = 0;
  e1.size_x = 32;
  e1.size_y = 32;
  e1.size_z = 32;
  m.extents.push_back(e1);

  brick_extent e2;
  e2.origin_x = 32;
  e2.origin_y = 0;
  e2.origin_z = 0;
  e2.size_x = 32;
  e2.size_y = 32;
  e2.size_z = 32;
  m.extents.push_back(e2);

  auto bytes = m.serialize();
  EXPECT_GT(bytes.size(), 0u);

  state_brick_manifest parsed;
  ASSERT_TRUE(state_brick_manifest::parse(bytes, parsed));

  EXPECT_EQ(parsed.version, 1u);
  EXPECT_EQ(parsed.chunk_size, 32u);
  EXPECT_EQ(parsed.total_size, 1024u);
  EXPECT_EQ(parsed.codec, "raw");
  EXPECT_EQ(parsed.content_digest, "abc123");
  EXPECT_TRUE(parsed.has_volume_header);
  EXPECT_EQ(parsed.vol_xdim, 64u);
  EXPECT_EQ(parsed.voxel_type, static_cast<std::uint32_t>(Float));
  EXPECT_EQ(parsed.chunks.size(), 2u);
  EXPECT_EQ(parsed.chunks[0], "chunk1");
  EXPECT_EQ(parsed.extents[0].size_x, 32u);
  EXPECT_EQ(parsed.extents[1].origin_x, 32u);
}

TEST_F(BrickManifestTest, ParseBadMagicFails) {
  std::vector<unsigned char> bad = {0, 0, 0, 0};
  state_brick_manifest m;
  EXPECT_FALSE(state_brick_manifest::parse(bad, m));
}

TEST_F(BrickManifestTest, NoVolumeHeader) {
  state_brick_manifest m;
  m.has_volume_header = false;
  m.chunks = {"d1"};
  m.chunk_bytes = {100};
  brick_extent e;
  e.element_begin = 0;
  e.element_end = 50;
  m.extents.push_back(e);

  auto bytes = m.serialize();
  state_brick_manifest parsed;
  ASSERT_TRUE(state_brick_manifest::parse(bytes, parsed));
  EXPECT_FALSE(parsed.has_volume_header);
  EXPECT_EQ(parsed.extents[0].element_end, 50u);
}

// ---- spatial query ----

TEST_F(BrickManifestTest, BricksInRegion) {
  state_brick_manifest m;
  m.has_volume_header = true;
  m.vol_xdim = 64;
  m.vol_ydim = 64;
  m.vol_zdim = 64;
  m.brick_xdim = 32;
  m.brick_ydim = 32;
  m.brick_zdim = 32;

  // 8 bricks in a 64^3 volume with 32^3 bricks.
  for (int bz = 0; bz < 2; ++bz) {
    for (int by = 0; by < 2; ++by) {
      for (int bx = 0; bx < 2; ++bx) {
        m.chunks.push_back("d" + std::to_string(bx + by * 2 + bz * 4));
        m.chunk_bytes.push_back(32768);
        brick_extent e;
        e.origin_x = bx * 32;
        e.origin_y = by * 32;
        e.origin_z = bz * 32;
        e.size_x = 32;
        e.size_y = 32;
        e.size_z = 32;
        m.extents.push_back(e);
      }
    }
  }

  // Query a region that covers the first octant only.
  auto r = m.bricks_in_region(0, 0, 0, 32, 32, 32);
  EXPECT_EQ(r.size(), 1u);
  EXPECT_EQ(r[0], 0u);

  // Query the full volume.
  auto all = m.bricks_in_region(0, 0, 0, 64, 64, 64);
  EXPECT_EQ(all.size(), 8u);

  // Query a 2x2x1 slab at z=0.
  auto slab = m.bricks_in_region(0, 0, 0, 64, 64, 1);
  EXPECT_EQ(slab.size(), 4u);

  // Query outside volume.
  auto empty = m.bricks_in_region(100, 100, 100, 200, 200, 200);
  EXPECT_EQ(empty.size(), 0u);
}

// ---- brick writer/reader ----

TEST_F(BrickManifestTest, WriteAndReadVolumeBricks) {
  const std::uint64_t xdim = 8, ydim = 8, zdim = 8;
  const std::uint32_t vs = sizeof(float);
  std::vector<unsigned char> voxels(xdim * ydim * zdim * vs);

  // Fill with known pattern.
  float *fp = reinterpret_cast<float *>(voxels.data());
  for (std::uint64_t i = 0; i < xdim * ydim * zdim; ++i)
    fp[i] = static_cast<float>(i);

  state_brick_writer writer(store, 4); // 4^3 bricks
  auto ref = writer.put_volume(voxels.data(), xdim, ydim, zdim, vs, Float);
  EXPECT_FALSE(ref.digest.empty());

  state_brick_reader reader(store);
  state_brick_manifest m;
  ASSERT_TRUE(reader.load_manifest(ref.digest, m));

  EXPECT_TRUE(m.has_volume_header);
  EXPECT_EQ(m.vol_xdim, 8u);
  EXPECT_EQ(m.brick_xdim, 4u);
  // 8/4 = 2 bricks per axis → 2^3 = 8 bricks.
  EXPECT_EQ(m.chunks.size(), 8u);
  EXPECT_EQ(m.extents.size(), 8u);

  // Verify spatial extents.
  EXPECT_EQ(m.extents[0].origin_x, 0u);
  EXPECT_EQ(m.extents[0].size_x, 4u);

  // Reassemble and verify full payload.
  std::vector<unsigned char> reassembled;
  ASSERT_TRUE(reader.get(m, reassembled));
  ASSERT_EQ(reassembled.size(), voxels.size());

  float *rp = reinterpret_cast<float *>(reassembled.data());
  for (std::uint64_t i = 0; i < xdim * ydim * zdim; ++i)
    EXPECT_FLOAT_EQ(rp[i], static_cast<float>(i)) << "at index " << i;
}

TEST_F(BrickManifestTest, WriteAndReadSingleBrick) {
  const std::uint64_t xdim = 4, ydim = 4, zdim = 4;
  const std::uint32_t vs = sizeof(float);
  std::vector<unsigned char> voxels(xdim * ydim * zdim * vs, 0);
  float *fp = reinterpret_cast<float *>(voxels.data());
  for (int i = 0; i < 64; ++i)
    fp[i] = static_cast<float>(i * 10);

  state_brick_writer writer(store, 4);
  auto ref = writer.put_volume(voxels.data(), xdim, ydim, zdim, vs, Float);

  state_brick_reader reader(store);
  state_brick_manifest m;
  ASSERT_TRUE(reader.load_manifest(ref.digest, m));
  EXPECT_EQ(m.chunks.size(), 1u); // 4/4 = 1 brick per axis

  std::vector<unsigned char> brick;
  ASSERT_TRUE(reader.get_brick(m, 0, brick));
  EXPECT_EQ(brick.size(), 4u * 4 * 4 * vs);
}

TEST_F(BrickManifestTest, GetRegion) {
  const std::uint64_t xdim = 8, ydim = 8, zdim = 8;
  const std::uint32_t vs = sizeof(float);
  std::vector<unsigned char> voxels(xdim * ydim * zdim * vs);
  float *fp = reinterpret_cast<float *>(voxels.data());
  for (std::uint64_t i = 0; i < xdim * ydim * zdim; ++i)
    fp[i] = static_cast<float>(i);

  state_brick_writer writer(store, 4);
  auto ref = writer.put_volume(voxels.data(), xdim, ydim, zdim, vs, Float);

  state_brick_reader reader(store);
  state_brick_manifest m;
  ASSERT_TRUE(reader.load_manifest(ref.digest, m));

  // Read the first 4x4x4 subregion.
  std::vector<unsigned char> region(4 * 4 * 4 * vs, 0);
  auto fetched = reader.get_region(m, region.data(), 4, 4, 4, vs, 0, 0, 0, 4, 4, 4);
  EXPECT_EQ(fetched, 1u);

  float *rp = reinterpret_cast<float *>(region.data());
  // The first voxel at (0,0,0) should be 0.
  EXPECT_FLOAT_EQ(rp[0], 0.0f);
  // Voxel at (3,3,3) in original = index 3 + 3*8 + 3*64 = 219.
  // In the subregion (3,3,3) = index 3 + 3*4 + 3*16 = 63.
  EXPECT_FLOAT_EQ(rp[63], 219.0f);
}

TEST_F(BrickManifestTest, MissingChunks) {
  state_brick_manifest m;
  m.chunks = {"nonexistent1", "nonexistent2"};

  state_brick_reader reader(store);
  auto missing = reader.missing_chunks(m);
  EXPECT_EQ(missing.size(), 2u);
}

TEST_F(BrickManifestTest, PutGeometry) {
  std::vector<unsigned char> data(1000, 42);
  state_brick_writer writer(store, 10); // 10^3 = 1000 chunk size
  auto ref = writer.put_geometry(data, 100);

  state_brick_reader reader(store);
  state_brick_manifest m;
  ASSERT_TRUE(reader.load_manifest(ref.digest, m));
  EXPECT_FALSE(m.has_volume_header);
  EXPECT_GE(m.chunks.size(), 1u);
  EXPECT_EQ(m.extents[0].element_begin, 0u);

  std::vector<unsigned char> reassembled;
  ASSERT_TRUE(reader.get(m, reassembled));
  EXPECT_EQ(reassembled, data);
}

TEST_F(BrickManifestTest, CompressedBricks) {
  const std::uint64_t xdim = 4, ydim = 4, zdim = 4;
  const std::uint32_t vs = 1;                                    // UChar
  std::vector<unsigned char> voxels(xdim * ydim * zdim * vs, 0); // all zeros → good RLE

  state_brick_writer writer(store, 4, &compression);
  auto ref = writer.put_volume(voxels.data(), xdim, ydim, zdim, vs, UChar, "rle");

  state_brick_reader reader(store, &compression);
  state_brick_manifest m;
  ASSERT_TRUE(reader.load_manifest(ref.digest, m));
  EXPECT_EQ(m.codec, "rle");

  std::vector<unsigned char> out;
  ASSERT_TRUE(reader.get(m, out));
  EXPECT_EQ(out, voxels);
}
