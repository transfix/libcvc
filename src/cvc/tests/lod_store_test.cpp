// lod_store_test -- scene.cvch5 round-trip for LOD pyramids.
//
// Build mesh + image pyramids, write them to an HDF5 file, read them back, and
// assert the rungs (positions, triangles, pixels, world-error ladder) survive
// bit-for-bit; then that read_lod_index() enumerates both assets and has_pyramid()
// honours the content hash. Then that in-memory writers and blob readers alive at
// the same time (also concurrently on a pool) each keep their own container, and
// that failures -- corrupt/empty/truncated blobs, missing or non-HDF5 files,
// tampered pixels -- surface as std::exception without a reader ever creating or
// truncating a file. HDF5-only (gated in CMake by CVC_USING_HDF5).

#include <H5Cpp.h>
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
#include <cvc/volume/hdf5_utils.h>
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
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

bool file_exists(const std::string &path) { return std::ifstream(path.c_str()).good(); }

std::string slurp(const std::string &path) {
  std::ifstream in(path.c_str(), std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}
void spit(const std::string &path, const std::string &bytes) {
  std::ofstream(path.c_str(), std::ios::binary | std::ios::trunc) << bytes;
}

// Replaces dataset `ds` of group `group` in `file` with a `rank`-D one of the
// given extent, writing `data` into it -- or, with data == nullptr, leaving it
// declared but never stored (HDF5 then reads it back as fill values).
void replace_dataset(cvc::app &ctx, const std::string &file, const std::string &group,
                     const char *ds, int rank, const hsize_t *dims, const H5::PredType &pt,
                     const void *data) {
  cvc::hdf5_utils::library_lock lock(ctx, file, "lod_store_test");
  H5::H5File f(file, H5F_ACC_RDWR);
  H5::Group g = f.openGroup(group);
  g.unlink(ds);
  H5::DataSet d = g.createDataSet(ds, pt, H5::DataSpace(rank, dims));
  if (data)
    d.write(data, pt);
}
void set_int_attr(cvc::app &ctx, const std::string &file, const std::string &group,
                  const char *attr, int value) {
  cvc::hdf5_utils::library_lock lock(ctx, file, "lod_store_test");
  H5::H5File f(file, H5F_ACC_RDWR);
  H5::Group g = f.openGroup(group);
  cvc::hdf5_utils::setAttribute<int>(g, attr, value);
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

// Runs `fn`, which must fail, and returns the what() of the std::exception it
// threw. H5::Exception is NOT a std::exception, so one leaking out of the store
// fails here instead of slipping past a caller's catch (const std::exception &).
template <class F> std::string std_error_of(F &&fn) {
  try {
    fn();
  } catch (const std::exception &e) {
    return e.what();
  } catch (...) {
    ADD_FAILURE() << "threw something that is not a std::exception";
    return std::string();
  }
  ADD_FAILURE() << "did not throw";
  return std::string();
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

TEST(LodStore, BlobCarriesNoStrayMemoryAfterAStringAttribute) {
  cvc::app ctx;
  geometry m = bumpy_grid(ctx, 6);
  const std::string hash = cvc::lod::mesh_content_hash(m);
  ASSERT_EQ(hash.size(), 64u);
  cvc::lod::scene_writer w(ctx);
  w.write_mesh_pyramid("m", tagged_pyramid(ctx, 1), hash);
  const std::vector<unsigned char> blob = w.to_blob();

  // source_hash is a fixed 256-byte string attribute stored inline, so the
  // blob holds the 64 hash characters and then 192 padding bytes. Those must
  // be zeros, not whatever followed the hash in the writer's memory: a blob
  // is meant to be sent over the network.
  const std::string bytes(blob.begin(), blob.end());
  const std::size_t at = bytes.find(hash);
  ASSERT_NE(at, std::string::npos);
  ASSERT_LE(at + 256, bytes.size());
  for (std::size_t i = at + hash.size(); i < at + 256; ++i)
    ASSERT_EQ(bytes[i], '\0') << "stray byte " << (i - at) << " after source_hash";
}

TEST(LodStore, MoveAssignReplacesTheContainer) {
  cvc::app ctx;
  cvc::lod::scene_writer a(ctx);
  a.write_mesh_pyramid(tagged_name(5), tagged_pyramid(ctx, 5));
  cvc::lod::scene_writer b(ctx);
  b.write_mesh_pyramid(tagged_name(6), tagged_pyramid(ctx, 6));
  a = std::move(b); // a's old container is closed; a now owns b's
  const std::vector<unsigned char> blob = a.to_blob();

  cvc::lod::scene_reader r(ctx, blob.data(), blob.size());
  cvc::lod::scene_reader moved(std::move(r));
  const auto c = contents(moved, 6);
  EXPECT_EQ(c.first, std::vector<std::string>{tagged_name(6)});
  EXPECT_EQ(c.second, fingerprint(tagged_pyramid(ctx, 6)));
}

// ── failures surface as std::exception, and a reader never damages a file ────

TEST(LodStore, CorruptBlobThrowsStdException) {
  cvc::app ctx;
  std::vector<unsigned char> garbage(4096);
  for (std::size_t i = 0; i < garbage.size(); ++i)
    garbage[i] = static_cast<unsigned char>((i * 131 + 7) & 0xff);
  EXPECT_FALSE(std_error_of([&] {
                 cvc::lod::scene_reader r(ctx, garbage.data(), garbage.size());
                 r.index();
               }).empty());

  EXPECT_FALSE(std_error_of([&] { cvc::lod::scene_reader r(ctx, nullptr, 0); }).empty());

  // A real image cut short: the superblock promises bytes that are not there.
  std::vector<unsigned char> cut = tagged_blob(ctx, 7);
  cut.resize(cut.size() / 2);
  EXPECT_FALSE(std_error_of([&] {
                 cvc::lod::scene_reader r(ctx, cut.data(), cut.size());
                 r.read_mesh_pyramid(tagged_name(7));
               }).empty());

  // An in-range lookup failure is reported the same way.
  const std::vector<unsigned char> blob = tagged_blob(ctx, 8);
  cvc::lod::scene_reader r(ctx, blob.data(), blob.size());
  EXPECT_FALSE(std_error_of([&] { r.read_mesh_pyramid("absent"); }).empty());
  EXPECT_FALSE(std_error_of([&] { r.read_image_pyramid(tagged_name(8)); }).empty());

  // ... including in a container with no /cvc group at all, where the lookup
  // must say the asset is absent rather than fail inside HDF5's path walk.
  std::vector<unsigned char> empty;
  {
    cvc::lod::scene_writer w(ctx);
    empty = w.to_blob();
  }
  cvc::lod::scene_reader re(ctx, empty.data(), empty.size());
  EXPECT_TRUE(re.index().empty());
  EXPECT_FALSE(re.has("absent"));
  const std::string msg = std_error_of([&] { re.read_mesh_pyramid("absent"); });
  EXPECT_NE(msg.find("no mesh pyramid 'absent'"), std::string::npos) << msg;
}

TEST(LodStore, MissingFileThrowsStdExceptionAndIsNotCreated) {
  cvc::app ctx;
  const std::string file = tmp_h5() + ".missing";
  std::remove(file.c_str());

  const std::string msg = std_error_of([&] { cvc::lod::scene_reader r(ctx, file); });
  EXPECT_FALSE(msg.empty());
  EXPECT_NE(msg.find(file), std::string::npos) << msg;
  EXPECT_FALSE(file_exists(file)) << "opening a reader created the file";

  EXPECT_FALSE(std_error_of([&] { cvc::lod::read_lod_index(ctx, file); }).empty());
  EXPECT_FALSE(std_error_of([&] { cvc::lod::read_mesh_pyramid(ctx, file, "x"); }).empty());
  EXPECT_FALSE(cvc::lod::has_pyramid(ctx, file, "x", ""));
  EXPECT_FALSE(file_exists(file));

  // A writer whose directory does not exist fails the same way.
  const std::string nodir = tmp_h5() + ".nodir/scene.cvch5";
  EXPECT_FALSE(std_error_of([&] { cvc::lod::scene_writer w(ctx, nodir); }).empty());
}

TEST(LodStore, ReaderNeverTruncatesANonHdf5File) {
  cvc::app ctx;
  const std::string file = tmp_h5() + ".txt";
  const std::string text = "not an HDF5 file -- a reader must leave it alone\n";
  {
    std::ofstream o(file.c_str(), std::ios::binary);
    o << text;
  }
  EXPECT_FALSE(std_error_of([&] { cvc::lod::scene_reader r(ctx, file); }).empty());
  EXPECT_FALSE(cvc::lod::has_pyramid(ctx, file, "x", ""));
  std::ifstream in(file.c_str(), std::ios::binary);
  const std::string after((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  EXPECT_EQ(after, text);
  in.close();
  std::remove(file.c_str());
}

TEST(LodStore, FileWriterAndReaderOnOnePath) {
  cvc::app ctx;
  const std::string file = tmp_h5();
  std::remove(file.c_str());
  cvc::lod::write_mesh_pyramid(ctx, file, tagged_name(1), tagged_pyramid(ctx, 1));

  {
    // A reader may join a live writer and sees what it has written.
    cvc::lod::scene_writer w(ctx, file);
    cvc::lod::scene_reader r(ctx, file);
    w.write_mesh_pyramid(tagged_name(2), tagged_pyramid(ctx, 2));
    EXPECT_EQ(r.index().size(), 2u);
    EXPECT_EQ(contents(r, 2).second, fingerprint(tagged_pyramid(ctx, 2)));
  }
  {
    // A live reader holds the file read-only: a writer is refused up front,
    // with a std::exception, rather than at its first write.
    cvc::lod::scene_reader r(ctx, file);
    const std::string msg = std_error_of([&] { cvc::lod::scene_writer w(ctx, file); });
    EXPECT_NE(msg.find("read-only"), std::string::npos) << msg;
    EXPECT_EQ(r.index().size(), 2u);
  }
  // Once the reader is gone the path is writable again.
  cvc::lod::write_mesh_pyramid(ctx, file, tagged_name(3), tagged_pyramid(ctx, 3));
  EXPECT_EQ(cvc::lod::read_lod_index(ctx, file).size(), 3u);
  std::remove(file.c_str());
}

TEST(LodStore, ImagePixelsThatDisagreeWithTheirSizeAreRejected) {
  cvc::app ctx;
  image src(16, 16, image::pixel_format::RGBA, image::data_type::u8);
  cvc::lod::image_pyramid pyr = cvc::lod::build_image_pyramid(src, {});
  const std::string file = tmp_h5();
  std::remove(file.c_str());
  cvc::lod::write_image_pyramid(ctx, file, "tex", pyr);

  // Tamper with rung 0's recorded size. The reader sizes the image from these
  // attributes and reads the whole pixels dataset into it, so trusting them
  // would overrun the buffer; it must refuse instead.
  auto set_rung0 = [&](const char *attr, int value) {
    cvc::hdf5_utils::library_lock lock(ctx, file, "lod_store_test");
    H5::H5File f(file, H5F_ACC_RDWR);
    H5::Group g = f.openGroup("/cvc/images/tex/lod/0");
    cvc::hdf5_utils::setAttribute<int>(g, attr, value);
  };
  set_rung0("w", 4);
  EXPECT_FALSE(std_error_of([&] { cvc::lod::read_image_pyramid(ctx, file, "tex"); }).empty());
  set_rung0("w", 16);
  EXPECT_EQ(cvc::lod::read_image_pyramid(ctx, file, "tex").rungs.size(), pyr.rungs.size());
  set_rung0("format", int(image::pixel_format::GRAY)); // 1 channel, 4 stored
  EXPECT_FALSE(std_error_of([&] { cvc::lod::read_image_pyramid(ctx, file, "tex"); }).empty());
  set_rung0("format", 42);
  EXPECT_FALSE(std_error_of([&] { cvc::lod::read_image_pyramid(ctx, file, "tex"); }).empty());
  std::remove(file.c_str());
}

TEST(LodStore, ImageSizesAreCheckedBeforeTheImageIsAllocated) {
  cvc::app ctx;
  image src(16, 16, image::pixel_format::RGBA, image::data_type::u8);
  cvc::lod::image_pyramid pyr = cvc::lod::build_image_pyramid(src, {});
  const std::string file = tmp_h5();
  std::remove(file.c_str());
  cvc::lod::write_image_pyramid(ctx, file, "tex", pyr);
  const std::string rung0 = "/cvc/images/tex/lod/0";

  // Rung 0 becomes a w x h RGBA image whose pixels dataset is h x dw x 4,
  // stored or only declared.
  auto make_rung0 = [&](int w, int h, hsize_t dw, bool stored) {
    const hsize_t dims[3] = {hsize_t(h < 0 ? 0 : h), dw, 4};
    std::vector<unsigned char> px;
    if (stored)
      px.assign(std::size_t(dims[0] * dims[1] * dims[2]), 7);
    replace_dataset(ctx, file, rung0, "pixels", 3, dims, H5::PredType::NATIVE_UINT8,
                    px.empty() ? nullptr : px.data());
    set_int_attr(ctx, file, rung0, "w", w);
    set_int_attr(ctx, file, rung0, "h", h);
  };
  auto read_error = [&] {
    return std_error_of([&] { cvc::lod::read_image_pyramid(ctx, file, "tex"); });
  };

  // A negative width over an empty dataset used to come back as an "image"
  // whose size_bytes() wrapped to ~2^64, crashing whatever trusted it next.
  make_rung0(-7, 5, 0, true);
  std::string msg = read_error();
  EXPECT_NE(msg.find("negative size"), std::string::npos) << msg;

  // A 64 MiB image declared over storage that was never written: the file is
  // tiny, so the reader must refuse before allocating the image, not read back
  // 64 MiB of fill values.
  make_rung0(4096, 4096, 4096, false);
  EXPECT_LT(slurp(file).size(), std::size_t(1) << 20);
  msg = read_error();
  EXPECT_NE(msg.find("declares more data than the file stores"), std::string::npos) << msg;

  // Attributes far larger than the stored pixels are refused by the extent
  // check, which now runs before the image is allocated.
  make_rung0(8, 8, 8, true);
  set_int_attr(ctx, file, rung0, "w", 16384);
  set_int_attr(ctx, file, rung0, "h", 16384);
  msg = read_error();
  EXPECT_NE(msg.find("pixels do not match"), std::string::npos) << msg;

  // A consistent, stored rung still reads back.
  make_rung0(8, 8, 8, true);
  const cvc::lod::image_pyramid rd = cvc::lod::read_image_pyramid(ctx, file, "tex");
  ASSERT_EQ(rd.rungs.size(), pyr.rungs.size());
  ASSERT_EQ(rd.rungs[0].width(), 8);
  ASSERT_EQ(rd.rungs[0].size_bytes(), 8u * 8u * 4u);
  EXPECT_EQ(rd.rungs[0].data()[0], 7);
  std::remove(file.c_str());
}

TEST(LodStore, MeshDatasetsMustBeStoredAndIndexInRange) {
  cvc::app ctx;
  const std::string file = tmp_h5();
  std::remove(file.c_str());
  const std::string rung0 = "/cvc/geometry/m/lod/0";
  auto read_error = [&] {
    return std_error_of([&] { cvc::lod::read_mesh_pyramid(ctx, file, "m"); });
  };

  // A million vertices declared but never stored.
  cvc::lod::write_mesh_pyramid(ctx, file, "m", tagged_pyramid(ctx, 1));
  const hsize_t huge[2] = {hsize_t(1) << 20, 3};
  replace_dataset(ctx, file, rung0, "points", 2, huge, H5::PredType::NATIVE_DOUBLE, nullptr);
  std::string msg = read_error();
  EXPECT_NE(msg.find("declares more data than the file stores"), std::string::npos) << msg;

  // A triangle naming a vertex one past the end.
  cvc::lod::write_mesh_pyramid(ctx, file, "m", tagged_pyramid(ctx, 1));
  const cvc::lod::mesh_pyramid good = cvc::lod::read_mesh_pyramid(ctx, file, "m");
  const std::uint64_t nv = good.rungs[0].num_points();
  const std::uint64_t tri[3] = {0, 1, nv};
  const hsize_t one[2] = {1, 3};
  replace_dataset(ctx, file, rung0, "tris", 2, one, H5::PredType::NATIVE_UINT64, tri);
  msg = read_error();
  EXPECT_NE(msg.find("triangle index out of range"), std::string::npos) << msg;

  // Rewritten, it reads back whole.
  cvc::lod::write_mesh_pyramid(ctx, file, "m", tagged_pyramid(ctx, 1));
  EXPECT_EQ(fingerprint(cvc::lod::read_mesh_pyramid(ctx, file, "m")),
            fingerprint(tagged_pyramid(ctx, 1)));
  std::remove(file.c_str());
}

TEST(LodStore, IndexChecksTheErrorLadderLengthBeforeSizingIt) {
  cvc::app ctx;
  const std::string file = tmp_h5();
  std::remove(file.c_str());
  cvc::lod::write_mesh_pyramid(ctx, file, "m", tagged_pyramid(ctx, 1)); // one rung

  // nrungs is only an attribute: sizing the ladder by it before checking the
  // stored world_error_m length let a corrupt value zero-fill gigabytes first.
  set_int_attr(ctx, file, "/cvc/geometry/m", "nrungs", 1 << 24);
  const std::string msg = std_error_of([&] { cvc::lod::read_lod_index(ctx, file); });
  EXPECT_NE(msg.find("1 world_error_m entries for nrungs 16777216"), std::string::npos) << msg;
  EXPECT_FALSE(std_error_of([&] { cvc::lod::read_mesh_pyramid(ctx, file, "m"); }).empty());

  set_int_attr(ctx, file, "/cvc/geometry/m", "nrungs", 1);
  const auto idx = cvc::lod::read_lod_index(ctx, file);
  ASSERT_EQ(idx.size(), 1u);
  EXPECT_EQ(idx[0].world_error_m, std::vector<double>{0.25});
  std::remove(file.c_str());
}

TEST(LodStore, WriterNeverTruncatesAnExistingFile) {
  cvc::app ctx;
  const geometry mesh = bumpy_grid(ctx, 6);

  // A mistyped output path naming some other file is refused, not replaced by
  // an empty scene -- by a writer, and so by a bake.
  const std::string png = tmp_h5() + ".png";
  const std::string text = "not an HDF5 file -- a writer must leave it alone\n";
  spit(png, text);
  std::string msg = std_error_of([&] { cvc::lod::scene_writer w(ctx, png); });
  EXPECT_NE(msg.find("will not overwrite"), std::string::npos) << msg;
  EXPECT_FALSE(std_error_of([&] { cvc::lod::bake_mesh_asset(ctx, png, "m", mesh); }).empty());
  EXPECT_EQ(slurp(png), text);
  std::remove(png.c_str());

  // A scene cut short keeps what is left of its assets for recovery.
  const std::string scene = tmp_h5();
  std::remove(scene.c_str());
  for (int tag = 1; tag <= 3; ++tag)
    cvc::lod::write_mesh_pyramid(ctx, scene, tagged_name(tag), tagged_pyramid(ctx, tag));
  const std::string whole = slurp(scene);
  ASSERT_GT(whole.size(), 100u);
  const std::string cut = whole.substr(0, whole.size() - 100);
  spit(scene, cut);
  EXPECT_FALSE(std_error_of([&] { cvc::lod::scene_writer w(ctx, scene); }).empty());
  EXPECT_FALSE(std_error_of([&] { cvc::lod::bake_mesh_asset(ctx, scene, "m", mesh); }).empty());
  EXPECT_EQ(slurp(scene), cut);

  // An intact scene is opened and added to, and a missing one is created.
  spit(scene, whole);
  EXPECT_TRUE(cvc::lod::bake_mesh_asset(ctx, scene, "m", mesh));
  EXPECT_EQ(cvc::lod::read_lod_index(ctx, scene).size(), 4u);
  std::remove(scene.c_str());
  EXPECT_TRUE(cvc::lod::bake_mesh_asset(ctx, scene, "m", mesh));
  EXPECT_EQ(cvc::lod::read_lod_index(ctx, scene).size(), 1u);
  std::remove(scene.c_str());
}
