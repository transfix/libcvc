// lod_store_test -- scene.cvch5 round-trip for LOD pyramids.
//
// Build mesh + image pyramids, write them to an HDF5 file, read them back, and
// assert the rungs (positions, triangles, pixels, world-error ladder) survive
// bit-for-bit; then that read_lod_index() enumerates both assets and has_pyramid()
// honours the content hash. Then that in-memory writers and blob readers alive at
// the same time (also concurrently on a pool) each keep their own container.
// HDF5-only (gated in CMake by CVC_USING_HDF5).

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/thread_pool.h>
#include <cvc/geometry/geometry.h>
#include <cvc/image/image.h>
#include <cvc/lod/pyramid.h>
#include <cvc/lod/store.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using cvc::geometry;
using cvc::image;

namespace {

geometry bumpy_grid(cvc::app &ctx, int n) {
  geometry g(ctx);
  geometry::points_t &P = g.points();
  geometry::uvs_t &UV = g.uvs();
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      P.push_back({double(i), double(j), 1.2 * std::sin(0.5 * i) * std::cos(0.7 * j)});
      UV.push_back({double(i) / n, double(j) / n});
    }
  geometry::tris_t &T = g.tris();
  auto id = [n](int i, int j) { return std::uint64_t(i * n + j); };
  for (int i = 0; i < n - 1; ++i)
    for (int j = 0; j < n - 1; ++j) {
      T.push_back({id(i, j), id(i + 1, j), id(i, j + 1)});
      T.push_back({id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)});
    }
  g.set_geometry_type(geometry::SURFACE_TRI);
  return g;
}

// Per-test, because ctest runs each discovered test in its own process in
// parallel: a name shared by every test lets one test's file clobber another's.
std::string tmp_h5() {
  return std::string(::testing::TempDir()) + "/lod_store_test_" +
         ::testing::UnitTest::GetInstance()->current_test_info()->name() + "_" +
         std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + ".cvch5";
}

// A one-rung mesh pyramid named by, and carrying, `tag` in every x coordinate,
// so a reader that hands back another container's bytes is caught by value and
// by name, not just by a count that might coincide.
cvc::lod::mesh_pyramid tagged_pyramid(cvc::app &ctx, int tag) {
  geometry g = bumpy_grid(ctx, 5 + tag % 4);
  for (auto &p : g.points())
    p[0] += 1000.0 * tag;
  cvc::lod::mesh_pyramid pyr;
  pyr.rungs.push_back(g);
  pyr.world_error_m.push_back(0.25 * tag);
  return pyr;
}
std::string tagged_name(int tag) { return "asset_" + std::to_string(tag); }

// Everything a round trip must preserve, flattened for one EXPECT_EQ.
std::vector<double> fingerprint(const cvc::lod::mesh_pyramid &pyr) {
  std::vector<double> f;
  for (std::size_t k = 0; k < pyr.rungs.size(); ++k) {
    f.push_back(pyr.world_error_m[k]);
    for (const auto &p : pyr.rungs[k].const_points())
      f.insert(f.end(), {p[0], p[1], p[2]});
    for (const auto &t : pyr.rungs[k].const_tris())
      f.insert(f.end(), {double(t[0]), double(t[1]), double(t[2])});
  }
  return f;
}

std::vector<unsigned char> tagged_blob(cvc::app &ctx, int tag) {
  cvc::lod::scene_writer w(ctx);
  w.write_mesh_pyramid(tagged_name(tag), tagged_pyramid(ctx, tag));
  return w.to_blob();
}

// What `r` holds, as (index names, fingerprint of the one tagged asset).
std::pair<std::vector<std::string>, std::vector<double>> contents(cvc::lod::scene_reader &r,
                                                                  int tag) {
  std::vector<std::string> names;
  for (const auto &e : r.index())
    names.push_back(e.name);
  std::vector<double> fp;
  if (r.has(tagged_name(tag)))
    fp = fingerprint(r.read_mesh_pyramid(tagged_name(tag)));
  return {names, fp};
}

} // namespace

TEST(LodStore, MeshPyramidRoundTrips) {
  cvc::app ctx;
  geometry src = bumpy_grid(ctx, 28);
  cvc::lod::mesh_pyramid pyr = cvc::lod::build_mesh_pyramid(src, {});
  ASSERT_GE(pyr.rungs.size(), 2u);
  const std::string hash = cvc::lod::mesh_content_hash(src);
  const std::string file = tmp_h5();
  std::remove(file.c_str());

  cvc::lod::write_mesh_pyramid(ctx, file, "buildings", pyr, hash);
  cvc::lod::mesh_pyramid rd = cvc::lod::read_mesh_pyramid(ctx, file, "buildings");

  ASSERT_EQ(rd.rungs.size(), pyr.rungs.size());
  for (std::size_t k = 0; k < pyr.rungs.size(); ++k) {
    EXPECT_DOUBLE_EQ(rd.world_error_m[k], pyr.world_error_m[k]) << "rung " << k;
    ASSERT_EQ(rd.rungs[k].num_points(), pyr.rungs[k].num_points()) << "rung " << k;
    ASSERT_EQ(rd.rungs[k].num_tris(), pyr.rungs[k].num_tris()) << "rung " << k;
    const auto &pa = pyr.rungs[k].const_points();
    const auto &pb = rd.rungs[k].const_points();
    for (std::size_t i = 0; i < pa.size(); ++i)
      for (int c = 0; c < 3; ++c)
        EXPECT_DOUBLE_EQ(pa[i][c], pb[i][c]);
    const auto &ta = pyr.rungs[k].const_tris();
    const auto &tb = rd.rungs[k].const_tris();
    for (std::size_t i = 0; i < ta.size(); ++i)
      for (int c = 0; c < 3; ++c)
        EXPECT_EQ(ta[i][c], tb[i][c]);
    // uv carried through
    EXPECT_EQ(rd.rungs[k].const_uvs().size(), pyr.rungs[k].const_uvs().size());
  }
  std::remove(file.c_str());
}

TEST(LodStore, ImagePyramidRoundTripsAndIndex) {
  cvc::app ctx;
  image src(16, 16, image::pixel_format::RGB, image::data_type::u8);
  unsigned char *d = src.data();
  for (std::size_t i = 0; i < src.size_bytes(); ++i)
    d[i] = static_cast<unsigned char>((i * 37) & 0xff);
  cvc::lod::image_pyramid pyr = cvc::lod::build_image_pyramid(src, {});
  ASSERT_GE(pyr.rungs.size(), 2u);
  const std::string ihash = cvc::lod::image_content_hash(src);
  const std::string file = tmp_h5();
  std::remove(file.c_str());

  // write a mesh too, so the index has two kinds
  geometry m = bumpy_grid(ctx, 16);
  cvc::lod::write_mesh_pyramid(ctx, file, "terrain", cvc::lod::build_mesh_pyramid(m, {}),
                               cvc::lod::mesh_content_hash(m));
  cvc::lod::write_image_pyramid(ctx, file, "satellite", pyr, ihash);

  cvc::lod::image_pyramid rd = cvc::lod::read_image_pyramid(ctx, file, "satellite");
  ASSERT_EQ(rd.rungs.size(), pyr.rungs.size());
  for (std::size_t k = 0; k < pyr.rungs.size(); ++k) {
    ASSERT_EQ(rd.rungs[k].width(), pyr.rungs[k].width());
    ASSERT_EQ(rd.rungs[k].height(), pyr.rungs[k].height());
    ASSERT_EQ(rd.rungs[k].size_bytes(), pyr.rungs[k].size_bytes());
    EXPECT_EQ(0, std::memcmp(rd.rungs[k].data(), pyr.rungs[k].data(), pyr.rungs[k].size_bytes()));
  }

  auto idx = cvc::lod::read_lod_index(ctx, file);
  EXPECT_EQ(idx.size(), 2u);
  bool sawImg = false, sawMesh = false;
  for (const auto &e : idx) {
    if (e.name == "satellite") {
      sawImg = true;
      EXPECT_EQ(e.kind, 'I');
      EXPECT_EQ(e.source_hash, ihash);
      EXPECT_EQ(int(e.world_error_m.size()), e.nrungs);
    }
    if (e.name == "terrain") {
      sawMesh = true;
      EXPECT_EQ(e.kind, 'M');
    }
  }
  EXPECT_TRUE(sawImg && sawMesh);

  EXPECT_TRUE(cvc::lod::has_pyramid(ctx, file, "satellite", ihash));
  EXPECT_FALSE(cvc::lod::has_pyramid(ctx, file, "satellite", "deadbeef")); // stale hash
  EXPECT_FALSE(cvc::lod::has_pyramid(ctx, file, "missing", ihash));
  std::remove(file.c_str());
}

TEST(LodStore, InMemoryBlobRoundTrips) {
  cvc::app ctx;
  geometry m = bumpy_grid(ctx, 22);
  cvc::lod::mesh_pyramid mp = cvc::lod::build_mesh_pyramid(m, {});
  image src(12, 12, image::pixel_format::RGBA, image::data_type::u8);
  {
    unsigned char *d = src.data();
    for (std::size_t i = 0; i < src.size_bytes(); ++i)
      d[i] = static_cast<unsigned char>((i * 53) & 0xff);
  }
  cvc::lod::image_pyramid ip = cvc::lod::build_image_pyramid(src, {});

  // Bake to a blob ENTIRELY in memory -- no file ever touched.
  std::vector<unsigned char> blob;
  {
    cvc::lod::scene_writer w(ctx);
    w.write_mesh_pyramid("buildings", mp, cvc::lod::mesh_content_hash(m));
    w.write_image_pyramid("satellite", ip, cvc::lod::image_content_hash(src));
    blob = w.to_blob();
  }
  ASSERT_GT(blob.size(), 8u);
  // The blob is a real HDF5 image: it opens with the HDF5 signature.
  EXPECT_EQ(blob[0], 0x89);
  EXPECT_EQ(blob[1], 'H');
  EXPECT_EQ(blob[2], 'D');
  EXPECT_EQ(blob[3], 'F');

  // Read it back from the blob as if it had been fetched over the network.
  cvc::lod::scene_reader r(ctx, blob.data(), blob.size());
  cvc::lod::mesh_pyramid rm = r.read_mesh_pyramid("buildings");
  ASSERT_EQ(rm.rungs.size(), mp.rungs.size());
  for (std::size_t k = 0; k < mp.rungs.size(); ++k) {
    EXPECT_EQ(rm.rungs[k].num_tris(), mp.rungs[k].num_tris());
    EXPECT_EQ(rm.rungs[k].num_points(), mp.rungs[k].num_points());
    EXPECT_DOUBLE_EQ(rm.world_error_m[k], mp.world_error_m[k]);
  }
  cvc::lod::image_pyramid ri = r.read_image_pyramid("satellite");
  ASSERT_EQ(ri.rungs.size(), ip.rungs.size());
  for (std::size_t k = 0; k < ip.rungs.size(); ++k) {
    ASSERT_EQ(ri.rungs[k].size_bytes(), ip.rungs[k].size_bytes());
    EXPECT_EQ(0, std::memcmp(ri.rungs[k].data(), ip.rungs[k].data(), ip.rungs[k].size_bytes()));
  }
  EXPECT_EQ(r.index().size(), 2u);
  EXPECT_TRUE(r.has("buildings", cvc::lod::mesh_content_hash(m)));
  EXPECT_FALSE(r.has("buildings", "nope"));
}

TEST(LodStore, BakeMeshAssetSkipsWhenCurrent) {
  cvc::app ctx;
  geometry m = bumpy_grid(ctx, 20);
  const std::string file = tmp_h5();
  std::remove(file.c_str());

  EXPECT_TRUE(cvc::lod::bake_mesh_asset(ctx, file, "buildings", m)); // first -> baked
  EXPECT_TRUE(cvc::lod::has_pyramid(ctx, file, "buildings", cvc::lod::mesh_content_hash(m)));
  EXPECT_GE(cvc::lod::read_mesh_pyramid(ctx, file, "buildings").rungs.size(), 2u);

  EXPECT_FALSE(cvc::lod::bake_mesh_asset(ctx, file, "buildings", m)); // same mesh -> skipped
  EXPECT_TRUE(cvc::lod::bake_mesh_asset(ctx, file, "buildings", m, cvc::lod::pyramid_params(),
                                        /*force=*/true)); // forced -> rebaked

  geometry m2 = bumpy_grid(ctx, 26);                                  // different content
  EXPECT_TRUE(cvc::lod::bake_mesh_asset(ctx, file, "buildings", m2)); // new hash -> rebaked
  std::remove(file.c_str());
}

// ── in-memory containers are independent (core-VFD file identity) ──────────

TEST(LodStore, TwoLiveInMemoryWritersKeepTheirOwnContent) {
  cvc::app ctx;
  // Both alive at once: each must be its own container, not a second handle on
  // (or a refused truncate of) the first.
  cvc::lod::scene_writer a(ctx);
  cvc::lod::scene_writer b(ctx);
  a.write_mesh_pyramid(tagged_name(1), tagged_pyramid(ctx, 1));
  b.write_mesh_pyramid(tagged_name(2), tagged_pyramid(ctx, 2));
  const std::vector<unsigned char> blob_a = a.to_blob();
  const std::vector<unsigned char> blob_b = b.to_blob();
  ASSERT_FALSE(blob_a.empty());
  ASSERT_FALSE(blob_b.empty());
  EXPECT_NE(blob_a, blob_b);

  cvc::lod::scene_reader ra(ctx, blob_a.data(), blob_a.size());
  const auto ca = contents(ra, 1);
  EXPECT_EQ(ca.first, std::vector<std::string>{tagged_name(1)});
  EXPECT_EQ(ca.second, fingerprint(tagged_pyramid(ctx, 1)));
  cvc::lod::scene_reader rb(ctx, blob_b.data(), blob_b.size());
  const auto cb = contents(rb, 2);
  EXPECT_EQ(cb.first, std::vector<std::string>{tagged_name(2)});
  EXPECT_EQ(cb.second, fingerprint(tagged_pyramid(ctx, 2)));
}

TEST(LodStore, TwoLiveBlobReadersDoNotAlias) {
  cvc::app ctx;
  const std::vector<unsigned char> blob_a = tagged_blob(ctx, 3);
  const std::vector<unsigned char> blob_b = tagged_blob(ctx, 4);

  // Open B while A is still open, then read both: B must see B's bytes, not the
  // container A already registered.
  cvc::lod::scene_reader ra(ctx, blob_a.data(), blob_a.size());
  cvc::lod::scene_reader rb(ctx, blob_b.data(), blob_b.size());
  const auto cb = contents(rb, 4);
  EXPECT_EQ(cb.first, std::vector<std::string>{tagged_name(4)});
  EXPECT_EQ(cb.second, fingerprint(tagged_pyramid(ctx, 4)));
  const auto ca = contents(ra, 3);
  EXPECT_EQ(ca.first, std::vector<std::string>{tagged_name(3)});
  EXPECT_EQ(ca.second, fingerprint(tagged_pyramid(ctx, 3)));

  // The reader keeps its own copy: the blob may go away once it is open.
  std::unique_ptr<cvc::lod::scene_reader> rc;
  {
    const std::vector<unsigned char> transient = tagged_blob(ctx, 5);
    rc.reset(new cvc::lod::scene_reader(ctx, transient.data(), transient.size()));
  }
  EXPECT_EQ(contents(*rc, 5).second, fingerprint(tagged_pyramid(ctx, 5)));
}

TEST(LodStore, ConcurrentWritersAndReadersOnAPoolMatchSerial) {
  cvc::app ctx;
  const int n = 16;

  // Serial reference: write, reopen, read back, one container at a time.
  std::vector<std::vector<double>> serial(n);
  for (int i = 0; i < n; ++i) {
    const std::vector<unsigned char> blob = tagged_blob(ctx, i);
    cvc::lod::scene_reader r(ctx, blob.data(), blob.size());
    serial[i] = contents(r, i).second;
    ASSERT_EQ(serial[i], fingerprint(tagged_pyramid(ctx, i))) << "tag " << i;
  }

  // The same on a pool, with many writers and then many readers alive at once
  // and closed concurrently.
  cvc::thread_pool pool(4);
  std::vector<std::vector<unsigned char>> blobs(n);
  pool.parallel_for(n, [&](int i) { blobs[i] = tagged_blob(ctx, i); });
  std::vector<std::unique_ptr<cvc::lod::scene_reader>> readers(n);
  pool.parallel_for(n, [&](int i) {
    readers[i].reset(new cvc::lod::scene_reader(ctx, blobs[i].data(), blobs[i].size()));
  });
  std::vector<std::pair<std::vector<std::string>, std::vector<double>>> got(n);
  pool.parallel_for(n, [&](int i) { got[i] = contents(*readers[i], i); });
  pool.parallel_for(n, [&](int i) { readers[i].reset(); });

  for (int i = 0; i < n; ++i) {
    EXPECT_EQ(got[i].first, std::vector<std::string>{tagged_name(i)}) << "tag " << i;
    EXPECT_EQ(got[i].second, serial[i]) << "tag " << i;
  }
}
