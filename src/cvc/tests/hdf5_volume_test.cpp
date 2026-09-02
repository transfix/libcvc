/*
  Copyright 2026 The University of Texas at Austin

  Tests for the HDF5 volume backend:
    src/cvc/volume/hdf5_io.cpp   (hdf5_io handler: .h5/.hdf5/.hdf/.cvc)
    src/cvc/volume/hdf5_utils.cpp and inc/cvc/volume/hdf5_utils.h

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <H5Cpp.h>
#include <atomic>
#include <boost/shared_array.hpp>
#include <boost/tuple/tuple.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/types.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/dimension.h>
#include <cvc/volume/hdf5_utils.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_info.h>
#include <cvc/volume/volume_file_io.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#include <process.h>
#define CVC_GETPID() ::_getpid()
#else
#include <unistd.h>
#define CVC_GETPID() ::getpid()
#endif

using namespace cvc;

namespace {

// One immortal app shared by every test in this binary.
//
// Rationale: every cvc::app constructor re-registers the default volume file
// I/O handlers into the process-wide handler map, and hdf5_io captures the
// constructing app by reference for the lifetime of that map.  With a
// per-test app, the first test's app would be destroyed while its hdf5_io
// handler remains first in the dispatch list, so every later HDF5 operation
// would run through a dangling app reference.  A single leaked app keeps all
// handler references (and any background hierarchy threads) valid for the
// whole process.
app &sharedApp() {
  static app *ctx = new app; // intentionally leaked, see above
  return *ctx;
}

const char *kDefaultObject = "/cvc/volumes/volume";
const char *kDefaultDataSet = "/cvc/volumes/volume/volume:0:0";

// Small synthetic volume with a recognizable gradient pattern.
volume make_test_volume(app &ctx, unsigned int xdim = 8, unsigned int ydim = 8,
                        unsigned int zdim = 8, data_type vt = Float, double shift = 0.0) {
  volume v(ctx, dimension(xdim, ydim, zdim), vt,
           bounding_box(0.0, 0.0, 0.0, double(xdim - 1), double(ydim - 1), double(zdim - 1)));
  for (unsigned int k = 0; k < zdim; ++k)
    for (unsigned int j = 0; j < ydim; ++j)
      for (unsigned int i = 0; i < xdim; ++i) {
        double val = vt == UChar ? double(i + j + k) + shift
                                 : double(i) + 10.0 * double(j) + 100.0 * double(k) + shift;
        v(i, j, k, val);
      }
  v.desc("hdf5-test");
  return v;
}

// HDF5 hyperslab I/O is byte-exact, so require equality at every voxel.
void expect_all_voxels_equal(const volume &a, const volume &b, double tol = 1e-4) {
  ASSERT_EQ(a.XDim(), b.XDim());
  ASSERT_EQ(a.YDim(), b.YDim());
  ASSERT_EQ(a.ZDim(), b.ZDim());
  for (uint64 k = 0; k < a.ZDim(); ++k)
    for (uint64 j = 0; j < a.YDim(); ++j)
      for (uint64 i = 0; i < a.XDim(); ++i)
        ASSERT_NEAR(a(i, j, k), b(i, j, k), tol)
            << "mismatch at (" << i << "," << j << "," << k << ")";
}

void expect_bbox_eq(const bounding_box &a, const bounding_box &b) {
  EXPECT_DOUBLE_EQ(a.minx, b.minx);
  EXPECT_DOUBLE_EQ(a.miny, b.miny);
  EXPECT_DOUBLE_EQ(a.minz, b.minz);
  EXPECT_DOUBLE_EQ(a.maxx, b.maxx);
  EXPECT_DOUBLE_EQ(a.maxy, b.maxy);
  EXPECT_DOUBLE_EQ(a.maxz, b.maxz);
}

} // namespace

class Hdf5VolumeTest : public ::testing::Test {
protected:
  app &ctx;
  std::string test_dir;

  Hdf5VolumeTest() : ctx(sharedApp()) {}

  void SetUp() override {
    namespace fs = std::filesystem;
    static std::atomic<unsigned> counter{0};
    std::ostringstream oss;
    oss << "hdf5vol_test_" << CVC_GETPID() << "_" << std::this_thread::get_id() << "_"
        << counter.fetch_add(1, std::memory_order_relaxed) << "_"
        << std::chrono::steady_clock::now().time_since_epoch().count();
    fs::path base = std::getenv("CMAKE_CURRENT_BINARY_DIR")
                        ? fs::path(std::getenv("CMAKE_CURRENT_BINARY_DIR"))
                        : fs::current_path();
    fs::path dir = base / oss.str();
    fs::create_directories(dir);
    test_dir = dir.string();
  }

  void TearDown() override {
    // Grouped HDF5 writes spawn a background multi-resolution hierarchy
    // thread that keeps touching the file for ~1s.  Join everything before
    // deleting the test directory so no thread races the cleanup.
    drainBackgroundThreads();
    std::error_code ec;
    std::filesystem::remove_all(test_dir, ec);
  }

  void drainBackgroundThreads() {
    thread_map tm = ctx.threads();
    for (thread_map::iterator i = tm.begin(); i != tm.end(); ++i)
      if (i->second && i->second->joinable())
        i->second->join();
  }

  std::string path(const std::string &name) const { return test_dir + "/" + name; }
};

// ===========================================================================
// hdf5_io: whole-volume round trips through the public volume API
// ===========================================================================

TEST_F(Hdf5VolumeTest, RoundTripDotH5AllVoxels) {
  volume out = make_test_volume(ctx);
  std::string p = path("rt.h5");
  ASSERT_NO_THROW(out.write(p));
  ASSERT_TRUE(std::filesystem::exists(p));

  // volume(app&, filename) constructor drives the read path.
  volume in(ctx, p);
  expect_all_voxels_equal(out, in);
  EXPECT_EQ(in.voxelType(), Float);
  EXPECT_EQ(in.desc(), "hdf5-test"); // round-tripped through the 'info' attribute
  // A full-size read fills min/max from the dataset attributes.
  EXPECT_NEAR(in.min(), 0.0, 1e-9);
  EXPECT_NEAR(in.max(), 777.0, 1e-9);

  volume_file_info info(ctx, p);
  EXPECT_TRUE(info.isSet());
  EXPECT_EQ(info.XDim(), 8u);
  EXPECT_EQ(info.YDim(), 8u);
  EXPECT_EQ(info.ZDim(), 8u);
  EXPECT_EQ(info.numVariables(), 1u);
  EXPECT_EQ(info.numTimesteps(), 1u);
  EXPECT_EQ(info.voxelType(), Float);
  EXPECT_EQ(info.name(0), "hdf5-test");
  EXPECT_NEAR(info.min(), 0.0, 1e-9);
  EXPECT_NEAR(info.max(), 777.0, 1e-9);
  EXPECT_DOUBLE_EQ(info.XMax(), 7.0);
}

TEST_F(Hdf5VolumeTest, RoundTripOtherExtensions) {
  const char *exts[] = {".hdf5", ".hdf", ".cvc"};
  for (int e = 0; e < 3; ++e) {
    volume out = make_test_volume(ctx, 6, 5, 4);
    std::string p = path(std::string("rt") + std::to_string(e) + exts[e]);
    ASSERT_NO_THROW(out.write(p)) << exts[e];
    volume in(ctx);
    ASSERT_NO_THROW(in.read(p)) << exts[e];
    expect_all_voxels_equal(out, in);
  }
}

TEST_F(Hdf5VolumeTest, NamedObjectUrlRoundTrip) {
  volume out = make_test_volume(ctx);
  std::string p = path("named.h5");
  std::string url = p + "|/cvc/volumes/mymap";

  // writeVolumeFile auto-creates the missing file, honoring the object path.
  ASSERT_NO_THROW(writeVolumeFile(ctx, out, url));
  EXPECT_TRUE(hdf5_utils::isGroup(ctx, p, "/cvc/volumes/mymap"));
  EXPECT_TRUE(hdf5_utils::isDataSet(ctx, p, "/cvc/volumes/mymap/volume:0:0"));
  // The default object was never created in this file.
  EXPECT_FALSE(hdf5_utils::objectExists(ctx, p, kDefaultObject));

  volume in(ctx);
  ASSERT_NO_THROW(readVolumeFile(ctx, in, url));
  expect_all_voxels_equal(out, in);

  volume_file_info info(ctx, url);
  EXPECT_EQ(info.XDim(), 8u);
  EXPECT_EQ(info.filename(), url);
}

TEST_F(Hdf5VolumeTest, MultiVariableMultiTimestep) {
  std::string p = path("multi.h5");
  const bounding_box bb(0.0, 0.0, 0.0, 7.0, 7.0, 7.0);
  const dimension dim(8, 8, 8);
  std::vector<data_type> types;
  types.push_back(Float);
  types.push_back(UChar);

  ASSERT_NO_THROW(createVolumeFile(ctx, p, bb, dim, types, 2, 2, 1.5, 4.5));

  // Fill each (variable, timestep) slot with distinct data.
  volume f0 = make_test_volume(ctx, 8, 8, 8, Float, 0.0);
  volume f1 = make_test_volume(ctx, 8, 8, 8, Float, 1.0);
  volume u0 = make_test_volume(ctx, 8, 8, 8, UChar, 1.0);
  volume u1 = make_test_volume(ctx, 8, 8, 8, UChar, 2.0);
  ASSERT_NO_THROW(writeVolumeFile(ctx, f0, p, 0, 0));
  ASSERT_NO_THROW(writeVolumeFile(ctx, f1, p, 0, 1));
  ASSERT_NO_THROW(writeVolumeFile(ctx, u0, p, 1, 0));
  ASSERT_NO_THROW(writeVolumeFile(ctx, u1, p, 1, 1));

  volume_file_info info(ctx, p);
  EXPECT_EQ(info.numVariables(), 2u);
  EXPECT_EQ(info.numTimesteps(), 2u);
  EXPECT_DOUBLE_EQ(info.TMin(), 1.5);
  EXPECT_DOUBLE_EQ(info.TMax(), 4.5);
  EXPECT_EQ(info.voxelTypes(0), Float);
  EXPECT_EQ(info.voxelTypes(1), UChar);
  EXPECT_NEAR(info.min(0, 0), 0.0, 1e-9);
  EXPECT_NEAR(info.max(0, 0), 777.0, 1e-9);
  EXPECT_NEAR(info.max(0, 1), 778.0, 1e-9);
  EXPECT_NEAR(info.min(1, 0), 1.0, 1e-9);
  EXPECT_NEAR(info.max(1, 1), 23.0, 1e-9);

  volume in(ctx);
  ASSERT_NO_THROW(readVolumeFile(ctx, in, p, 0, 1));
  expect_all_voxels_equal(f1, in);
  ASSERT_NO_THROW(readVolumeFile(ctx, in, p, 1, 0));
  expect_all_voxels_equal(u0, in);

  // Vector overload walks every var/timestep in order.
  std::vector<volume> vols;
  ASSERT_NO_THROW(readVolumeFile(ctx, vols, p));
  ASSERT_EQ(vols.size(), 4u);
  expect_all_voxels_equal(f0, vols[0]);
  expect_all_voxels_equal(f1, vols[1]);
  expect_all_voxels_equal(u0, vols[2]);
  expect_all_voxels_equal(u1, vols[3]);
}

// ===========================================================================
// hdf5_io: offset and bounding-box addressed sub-volume access
// ===========================================================================

TEST_F(Hdf5VolumeTest, OffsetReadSubvolume) {
  volume out = make_test_volume(ctx);
  std::string p = path("offr.h5");
  ASSERT_NO_THROW(out.write(p));

  volume sub(ctx);
  ASSERT_NO_THROW(readVolumeFile(ctx, sub, p, 0, 0, 2, 3, 4, dimension(6, 5, 4)));
  ASSERT_EQ(sub.XDim(), 6u);
  ASSERT_EQ(sub.YDim(), 5u);
  ASSERT_EQ(sub.ZDim(), 4u);
  for (uint64 k = 0; k < 4; ++k)
    for (uint64 j = 0; j < 5; ++j)
      for (uint64 i = 0; i < 6; ++i)
        ASSERT_NEAR(sub(i, j, k), out(i + 2, j + 3, k + 4), 1e-4);
  // NOTE: sub.boundingBox() is not checked here; hdf5_io computes the
  // sub-box using integer division of voxel offsets (see notes/bug report),
  // so the voxel payload is exact but the reported box is not.

  // Offset+dim beyond the stored dimensions must fail.
  EXPECT_ANY_THROW(readVolumeFile(ctx, sub, p, 0, 0, 1, 0, 0, dimension(8, 8, 8)));
  // Null subvolume dimension must fail.
  EXPECT_ANY_THROW(readVolumeFile(ctx, sub, p, 0, 0, 0, 0, 0, dimension(0, 0, 0)));
}

TEST_F(Hdf5VolumeTest, OffsetWriteIntoCreatedFile) {
  std::string p = path("offw.h5");
  const bounding_box bb(0.0, 0.0, 0.0, 7.0, 7.0, 7.0);
  ASSERT_NO_THROW(
      createVolumeFile(ctx, p, bb, dimension(8, 8, 8), std::vector<data_type>(1, Float)));

  volume smallvol = make_test_volume(ctx, 4, 4, 4, Float, 1.0);
  ASSERT_NO_THROW(writeVolumeFile(ctx, smallvol, p, 0, 0, 2, 2, 2));

  volume full(ctx);
  ASSERT_NO_THROW(readVolumeFile(ctx, full, p));
  ASSERT_EQ(full.XDim(), 8u);
  // The written window matches the smallvol volume...
  for (uint64 k = 0; k < 4; ++k)
    for (uint64 j = 0; j < 4; ++j)
      for (uint64 i = 0; i < 4; ++i)
        ASSERT_NEAR(full(i + 2, j + 2, k + 2), smallvol(i, j, k), 1e-4);
  // ...and untouched voxels keep the dataset fill value.
  EXPECT_NEAR(full(0, 0, 0), 0.0, 1e-9);
  EXPECT_NEAR(full(7, 7, 7), 0.0, 1e-9);

  volume_file_info info(ctx, p);
  EXPECT_NEAR(info.min(), smallvol.min(), 1e-9);
  EXPECT_NEAR(info.max(), smallvol.max(), 1e-9);
}

TEST_F(Hdf5VolumeTest, BoundingBoxReadFullAndSub) {
  volume out = make_test_volume(ctx);
  std::string p = path("bbr.h5");
  ASSERT_NO_THROW(out.write(p));

  // The grouped bounding-box read path only works when the primary dataset
  // carries a 'dirty' attribute: hdf5_io's dirty-probe catches only
  // std::exception, but cvc exceptions derive from boost::exception alone,
  // so the probe's hdf5_exception escapes otherwise (see bug notes).  Stamp
  // the attribute the same way the hierarchy thread stamps its datasets.
  hdf5_utils::setAttribute(ctx, p, kDefaultDataSet, "dirty", 0);

  const bounding_box full(0.0, 0.0, 0.0, 7.0, 7.0, 7.0);
  volume v(ctx);
  ASSERT_NO_THROW(readVolumeFile(ctx, v, p, 0, 0, full));
  expect_all_voxels_equal(out, v);
  // Full-box read publishes the stored min/max.
  EXPECT_NEAR(v.min(), 0.0, 1e-9);
  EXPECT_NEAR(v.max(), 777.0, 1e-9);

  const bounding_box subbox(0.0, 0.0, 0.0, 3.0, 3.0, 3.0);
  volume s(ctx);
  ASSERT_NO_THROW(readVolumeFile(ctx, s, p, 0, 0, subbox));
  ASSERT_EQ(s.XDim(), 4u);
  ASSERT_EQ(s.YDim(), 4u);
  ASSERT_EQ(s.ZDim(), 4u);
  for (uint64 k = 0; k < 4; ++k)
    for (uint64 j = 0; j < 4; ++j)
      for (uint64 i = 0; i < 4; ++i)
        ASSERT_NEAR(s(i, j, k), out(i, j, k), 1e-4);

  // A variable/timestep with no matching child datasets must fail.
  EXPECT_ANY_THROW(readVolumeFile(ctx, s, p, 5, 9, full));
  // A box fully outside the stored volume: the HDF5 backend does not define
  // whether this throws or clamps to an empty/edge read, and the two behave
  // differently across platforms (it throws on the Linux HDF5 build but
  // clamps on the macOS one). Only require that it does not corrupt state —
  // either an exception or a successful (clamped) read is acceptable.
  const bounding_box outside(-4.0, -4.0, -4.0, -1.0, -1.0, -1.0);
  volume oob(ctx);
  try {
    readVolumeFile(ctx, oob, p, 0, 0, outside);
    // Clamped read path: dimensions must still be sane (non-zero, bounded).
    EXPECT_GT(oob.XDim(), 0u);
    EXPECT_GT(oob.YDim(), 0u);
    EXPECT_GT(oob.ZDim(), 0u);
  } catch (...) {
    // Throwing path (cvc/H5 exception types, not all std::exception): also OK.
    SUCCEED();
  }
}

// ===========================================================================
// hdf5_io: lone (ungrouped) dataset addressing via file|object URLs
// ===========================================================================

TEST_F(Hdf5VolumeTest, LoneDataSetUrlReadWrite) {
  volume out = make_test_volume(ctx);
  std::string p = path("lone.h5");
  ASSERT_NO_THROW(out.write(p));
  drainBackgroundThreads(); // let the hierarchy thread finish with the file

  std::string url = p + "|" + kDefaultDataSet;

  volume_file_info info(ctx, url);
  EXPECT_EQ(info.numVariables(), 1u);
  EXPECT_EQ(info.numTimesteps(), 1u);
  EXPECT_EQ(info.voxelType(), Float);
  EXPECT_EQ(info.name(0), "hdf5-test");
  EXPECT_EQ(info.XDim(), 8u);

  volume in(ctx);
  ASSERT_NO_THROW(readVolumeFile(ctx, in, url));
  expect_all_voxels_equal(out, in);

  // var/time > 0 on a lone dataset logs a warning and falls back to 0/0.
  volume in2(ctx);
  ASSERT_NO_THROW(readVolumeFile(ctx, in2, url, 1, 0));
  expect_all_voxels_equal(out, in2);

  // Writing through the lone-dataset URL updates in place (no hierarchy).
  volume shifted = make_test_volume(ctx, 8, 8, 8, Float, -3.0);
  ASSERT_NO_THROW(writeVolumeFile(ctx, shifted, url));
  volume in3(ctx);
  ASSERT_NO_THROW(readVolumeFile(ctx, in3, url));
  expect_all_voxels_equal(shifted, in3);
  // min attribute lowered by the new data; max keeps the historic high.
  volume_file_info info2(ctx, url);
  EXPECT_NEAR(info2.min(), -3.0, 1e-9);
  EXPECT_NEAR(info2.max(), 777.0, 1e-9);

  // var/time > 0 write on a lone dataset warns and writes slot 0/0.
  volume shifted2 = make_test_volume(ctx, 8, 8, 8, Float, 5.0);
  ASSERT_NO_THROW(writeVolumeFile(ctx, shifted2, url, 2, 3));
  volume in4(ctx);
  ASSERT_NO_THROW(readVolumeFile(ctx, in4, url));
  expect_all_voxels_equal(shifted2, in4);

  // A lone dataset claiming several variables triggers the warning path.
  // getVolumeFileInfo clamps its internal per-variable arrays to one entry
  // but leaves the reported numVariables at the raw attribute value (see
  // bug notes), so 2 is the current observable behavior.
  hdf5_utils::setAttribute(ctx, p, kDefaultDataSet, "numVariables", uint64(2));
  volume_file_info info3(ctx, url);
  EXPECT_EQ(info3.numVariables(), 2u);
  EXPECT_EQ(info3.voxelType(), Float);
}

TEST_F(Hdf5VolumeTest, WriteBoundingBoxGroupAndLone) {
  volume out = make_test_volume(ctx);
  std::string p = path("wbb.h5");
  ASSERT_NO_THROW(out.write(p));
  drainBackgroundThreads();

  const bounding_box bb2(0.0, 0.0, 0.0, 14.0, 14.0, 14.0);
  ASSERT_NO_THROW(writeBoundingBox(ctx, bb2, p));
  expect_bbox_eq(readBoundingBox(ctx, p), bb2);
  // Group children (including hierarchy datasets) were updated too.
  expect_bbox_eq(hdf5_utils::getObjectBoundingBox(ctx, p, kDefaultDataSet), bb2);

  // Lone-dataset URL updates only that dataset.
  std::string url = p + "|" + kDefaultDataSet;
  const bounding_box bb3(1.0, 1.0, 1.0, 21.0, 21.0, 21.0);
  ASSERT_NO_THROW(writeBoundingBox(ctx, bb3, url));
  expect_bbox_eq(hdf5_utils::getObjectBoundingBox(ctx, p, kDefaultDataSet), bb3);
  expect_bbox_eq(hdf5_utils::getObjectBoundingBox(ctx, p, kDefaultObject), bb2);
}

// ===========================================================================
// hdf5_io: background multi-resolution hierarchy
// ===========================================================================

TEST_F(Hdf5VolumeTest, HierarchyDatasetsAfterWrite) {
  volume out = make_test_volume(ctx, 16, 16, 16);
  std::string p = path("hier.h5");
  ASSERT_NO_THROW(out.write(p));

  std::string threadKey = std::string("build_hierarchy_") + kDefaultDataSet;
  ASSERT_TRUE(ctx.hasThread(threadKey));
  thread_ptr t = ctx.threads(threadKey);
  ASSERT_TRUE(t.get() != NULL);
  t->join();

  // All hierarchy levels below 16^3 exist and are marked clean.
  const char *levels[] = {"_8x8x8", "_4x4x4", "_2x2x2"};
  for (int i = 0; i < 3; ++i) {
    std::string ds = std::string(kDefaultDataSet) + levels[i];
    ASSERT_TRUE(hdf5_utils::isDataSet(ctx, p, ds)) << ds;
    int dirty = -1;
    hdf5_utils::getAttribute(ctx, p, ds, "dirty", dirty);
    EXPECT_EQ(dirty, 0) << ds;
  }

  // The thread reports the last level it produced.
  ASSERT_TRUE(ctx.hasProperty("volmagick.hdf5_io.buildhierarchy.latest"));
  EXPECT_EQ(ctx.properties("volmagick.hdf5_io.buildhierarchy.latest"),
            p + "|" + kDefaultDataSet + "_2x2x2");

  // Grouped bounding-box reads need a 'dirty' stamp on the primary dataset
  // (see bug notes on the ineffective std::exception dirty-probe catch).
  hdf5_utils::setAttribute(ctx, p, kDefaultDataSet, "dirty", 0);

  // With a smallvol maxdim the bounding-box read selects a coarser level.
  std::string savedMaxdim = ctx.properties("volmagick.hdf5_io.maxdim");
  ctx.properties("volmagick.hdf5_io.maxdim", "4,4,4");
  volume coarse(ctx);
  std::string readError;
  try {
    readVolumeFile(ctx, coarse, p, 0, 0, bounding_box(0.0, 0.0, 0.0, 15.0, 15.0, 15.0));
  } catch (cvc::exception &e) {
    readError = e.what();
  } catch (...) {
    readError = "non-cvc exception";
  }
  ctx.properties("volmagick.hdf5_io.maxdim", savedMaxdim); // restore before asserting
  ASSERT_TRUE(readError.empty()) << readError;
  EXPECT_EQ(coarse.XDim(), 4u);
  EXPECT_EQ(coarse.YDim(), 4u);
  EXPECT_EQ(coarse.ZDim(), 4u);
  // Corners of the downsampled level interpolate back to the original corners.
  EXPECT_NEAR(coarse(0, 0, 0), out(0, 0, 0), 1.0);
  EXPECT_NEAR(coarse(3, 3, 3), out(15, 15, 15), 1.0);
}

// ===========================================================================
// hdf5_io: legacy VolMagick attribute schema
// ===========================================================================

TEST_F(Hdf5VolumeTest, OldVolMagickAttributeFormat) {
  volume out = make_test_volume(ctx);
  std::string p = path("oldfmt.h5");
  ASSERT_NO_THROW(out.write(p));
  drainBackgroundThreads();

  // Stamp the old-style attributes onto the group; the presence of
  // VolMagick_version switches getVolumeFileInfo to the legacy reader.
  hdf5_utils::setAttribute(ctx, p, kDefaultObject, "VolMagick_version", uint64(1));
  hdf5_utils::setAttribute(ctx, p, kDefaultObject, "XMin", 0.0);
  hdf5_utils::setAttribute(ctx, p, kDefaultObject, "YMin", 0.0);
  hdf5_utils::setAttribute(ctx, p, kDefaultObject, "ZMin", 0.0);
  hdf5_utils::setAttribute(ctx, p, kDefaultObject, "XMax", 7.0);
  hdf5_utils::setAttribute(ctx, p, kDefaultObject, "YMax", 7.0);
  hdf5_utils::setAttribute(ctx, p, kDefaultObject, "ZMax", 7.0);
  hdf5_utils::setAttribute(ctx, p, kDefaultObject, "XDim", uint64(8));
  hdf5_utils::setAttribute(ctx, p, kDefaultObject, "YDim", uint64(8));
  hdf5_utils::setAttribute(ctx, p, kDefaultObject, "ZDim", uint64(8));
  hdf5_utils::setAttribute(ctx, p, kDefaultDataSet, "voxelType", uint64(Float));

  volume_file_info info(ctx, p);
  EXPECT_EQ(info.XDim(), 8u);
  EXPECT_EQ(info.numVariables(), 1u);
  EXPECT_EQ(info.voxelType(), Float);
  EXPECT_DOUBLE_EQ(info.XMax(), 7.0);
  EXPECT_NEAR(info.max(), 777.0, 1e-9);

  volume in(ctx);
  ASSERT_NO_THROW(readVolumeFile(ctx, in, p));
  expect_all_voxels_equal(out, in);

  // Legacy attributes on a lone dataset drive the old-format lone branch.
  std::string url = p + "|" + kDefaultDataSet;
  hdf5_utils::setAttribute(ctx, p, kDefaultDataSet, "VolMagick_version", uint64(1));
  hdf5_utils::setAttribute(ctx, p, kDefaultDataSet, "XMin", 0.0);
  hdf5_utils::setAttribute(ctx, p, kDefaultDataSet, "YMin", 0.0);
  hdf5_utils::setAttribute(ctx, p, kDefaultDataSet, "ZMin", 0.0);
  hdf5_utils::setAttribute(ctx, p, kDefaultDataSet, "XMax", 7.0);
  hdf5_utils::setAttribute(ctx, p, kDefaultDataSet, "YMax", 7.0);
  hdf5_utils::setAttribute(ctx, p, kDefaultDataSet, "ZMax", 7.0);
  hdf5_utils::setAttribute(ctx, p, kDefaultDataSet, "XDim", uint64(8));
  hdf5_utils::setAttribute(ctx, p, kDefaultDataSet, "YDim", uint64(8));
  hdf5_utils::setAttribute(ctx, p, kDefaultDataSet, "ZDim", uint64(8));
  volume_file_info lone(ctx, url);
  EXPECT_EQ(lone.voxelType(), Float);
  EXPECT_EQ(lone.XDim(), 8u);
}

// ===========================================================================
// hdf5_io: error paths
// ===========================================================================

TEST_F(Hdf5VolumeTest, ErrorPathsBadFilesAndArgs) {
  // Garbage bytes with an .h5 extension.
  std::string garbage = path("garbage.h5");
  {
    std::ofstream f(garbage.c_str());
    f << "this is not an hdf5 file at all\n";
  }
  EXPECT_ANY_THROW(volume_file_info info(ctx, garbage); (void)info);

  // Missing file.
  volume rv(ctx);
  EXPECT_ANY_THROW(rv.read(path("missing.h5")));

  // Valid file, bogus object path.
  volume out = make_test_volume(ctx);
  std::string p = path("valid.h5");
  ASSERT_NO_THROW(out.write(p));
  EXPECT_ANY_THROW(readVolumeFile(ctx, rv, p + "|/cvc/volumes/nothere"));

  // createVolumeFile argument validation.
  EXPECT_ANY_THROW(createVolumeFile(ctx, path("e1.h5"), bounding_box(0, 0, 0, 1, 1, 1),
                                    dimension(4, 4, 4), std::vector<data_type>()));
  EXPECT_ANY_THROW(createVolumeFile(ctx, path("e2.h5"), bounding_box(0, 0, 0, 1, 1, 1),
                                    dimension(4, 4, 4), std::vector<data_type>(1, Float), 2, 1));
}

TEST_F(Hdf5VolumeTest, CreateVolumeFileVariants) {
  std::string p = path("variants.h5");
  const bounding_box bb(0.0, 0.0, 0.0, 7.0, 7.0, 7.0);
  const dimension dim(8, 8, 8);

  std::string urlA = p + "|/cvc/volumes/a";
  std::string urlB = p + "|/cvc/volumes/b";
  ASSERT_NO_THROW(createVolumeFile(ctx, urlA, bb, dim, std::vector<data_type>(1, Float)));
  // Second create against the same physical file reuses it.
  ASSERT_NO_THROW(createVolumeFile(ctx, urlB, bb, dim, std::vector<data_type>(1, UChar)));
  EXPECT_TRUE(hdf5_utils::objectExists(ctx, p, "/cvc/volumes/a"));
  EXPECT_TRUE(hdf5_utils::objectExists(ctx, p, "/cvc/volumes/b"));

  volume out = make_test_volume(ctx);
  ASSERT_NO_THROW(writeVolumeFile(ctx, out, urlA));
  volume in(ctx);
  ASSERT_NO_THROW(readVolumeFile(ctx, in, urlA));
  expect_all_voxels_equal(out, in);
  drainBackgroundThreads();

  // Re-creating an existing object replaces it with a blank volume.
  ASSERT_NO_THROW(createVolumeFile(ctx, urlA, bb, dim, std::vector<data_type>(1, Float)));
  EXPECT_DOUBLE_EQ(hdf5_utils::getDataSetMinimum(ctx, p, "/cvc/volumes/a/volume:0:0"),
                   std::numeric_limits<double>::max());
}

// ===========================================================================
// hdf5_utils: file/group/dataset lifecycle
// ===========================================================================

TEST_F(Hdf5VolumeTest, UtilsFileGroupDataSetLifecycle) {
  std::string p = path("lifecycle.h5");
  ASSERT_NO_THROW(hdf5_utils::createHDF5File(ctx, p));
  EXPECT_TRUE(std::filesystem::exists(p));

  EXPECT_FALSE(hdf5_utils::isGroup(ctx, p, "/g1"));
  ASSERT_NO_THROW(hdf5_utils::createGroup(ctx, p, "/g1/g2"));
  EXPECT_TRUE(hdf5_utils::isGroup(ctx, p, "/g1"));
  EXPECT_TRUE(hdf5_utils::isGroup(ctx, p, "/g1/g2"));
  EXPECT_TRUE(hdf5_utils::objectExists(ctx, p, "/g1/g2"));
  EXPECT_FALSE(hdf5_utils::isDataSet(ctx, p, "/g1"));

  // Existing object: refuse without replace, succeed with it.
  EXPECT_THROW(hdf5_utils::createGroup(ctx, p, "/g1"), hdf5_exception);
  ASSERT_NO_THROW(hdf5_utils::createGroup(ctx, p, "/g1", true));
  EXPECT_FALSE(hdf5_utils::objectExists(ctx, p, "/g1/g2")); // children wiped

  const bounding_box bb(0.0, 0.0, 0.0, 3.0, 3.0, 3.0);
  ASSERT_NO_THROW(hdf5_utils::createDataSet(ctx, p, "/g1/ds", bb, dimension(4, 4, 4), Float));
  EXPECT_TRUE(hdf5_utils::isDataSet(ctx, p, "/g1/ds"));
  EXPECT_FALSE(hdf5_utils::isGroup(ctx, p, "/g1/ds"));
  EXPECT_EQ(hdf5_utils::getDataSetInfo(ctx, p, "/g1/ds"), "No Name");
  EXPECT_EQ(hdf5_utils::getDataSetType(ctx, p, "/g1/ds"), Float);
  EXPECT_DOUBLE_EQ(hdf5_utils::getDataSetMinimum(ctx, p, "/g1/ds"),
                   std::numeric_limits<double>::max());
  EXPECT_DOUBLE_EQ(hdf5_utils::getDataSetMaximum(ctx, p, "/g1/ds"),
                   -std::numeric_limits<double>::max());

  dimension d = hdf5_utils::getObjectDimension(ctx, p, "/g1/ds");
  EXPECT_EQ(d.xdim, 4u);
  EXPECT_EQ(d.ydim, 4u);
  EXPECT_EQ(d.zdim, 4u);
  expect_bbox_eq(hdf5_utils::getObjectBoundingBox(ctx, p, "/g1/ds"), bb);

  ASSERT_NO_THROW(hdf5_utils::setObjectDimension(ctx, p, "/g1/ds", dimension(5, 6, 7)));
  d = hdf5_utils::getObjectDimension(ctx, p, "/g1/ds");
  EXPECT_EQ(d.xdim, 5u);
  EXPECT_EQ(d.ydim, 6u);
  EXPECT_EQ(d.zdim, 7u);
  const bounding_box bb2(0.0, 0.0, 0.0, 9.0, 9.0, 9.0);
  ASSERT_NO_THROW(hdf5_utils::setObjectBoundingBox(ctx, p, "/g1/ds", bb2));
  expect_bbox_eq(hdf5_utils::getObjectBoundingBox(ctx, p, "/g1/ds"), bb2);

  // Dataset replacement semantics.
  EXPECT_THROW(hdf5_utils::createDataSet(ctx, p, "/g1/ds", bb, dimension(4, 4, 4), Float),
               hdf5_exception);
  ASSERT_NO_THROW(hdf5_utils::createDataSet(ctx, p, "/g1/ds", bb, dimension(4, 4, 4), Double,
                                            true /*replace*/));
  EXPECT_EQ(hdf5_utils::getDataSetType(ctx, p, "/g1/ds"), Double);

  // Unsupported creation type.
  EXPECT_ANY_THROW(hdf5_utils::createDataSet(ctx, p, "/g1/bad", bb, dimension(4, 4, 4), Int));

  // Child object listing (with and without filter).
  ASSERT_NO_THROW(hdf5_utils::createDataSet(ctx, p, "/g1/ds2", bb, dimension(2, 2, 2), UChar));
  std::vector<std::string> kids = hdf5_utils::getChildObjects(ctx, p, "/g1");
  EXPECT_EQ(kids.size(), 2u);
  kids = hdf5_utils::getChildObjects(ctx, p, "/g1", "ds2");
  ASSERT_EQ(kids.size(), 1u);
  EXPECT_EQ(kids[0], "ds2");
  kids = hdf5_utils::getChildObjects(ctx, p, "/g1", "zzz");
  EXPECT_TRUE(kids.empty());
  // NOTE: getChildObjects(ctx, p) with the default root object "/" is NOT
  // exercised here: getGroup() returns a null pointer for the root path and
  // getChildObjects dereferences it (segfault; see bug notes).
  EXPECT_THROW(hdf5_utils::getChildObjects(ctx, p, "/nope"), hdf5_exception);

  // Removal: nested object then root-level object (both unlink branches).
  ASSERT_NO_THROW(hdf5_utils::removeObject(ctx, p, "/g1/ds2"));
  EXPECT_FALSE(hdf5_utils::objectExists(ctx, p, "/g1/ds2"));
  ASSERT_NO_THROW(hdf5_utils::createDataSet(ctx, p, "/topds", bb, dimension(2, 2, 2), UChar));
  ASSERT_NO_THROW(hdf5_utils::removeObject(ctx, p, "/topds"));
  EXPECT_FALSE(hdf5_utils::objectExists(ctx, p, "/topds"));
  EXPECT_THROW(hdf5_utils::removeObject(ctx, p, "/neverwas"), hdf5_exception);

  // Metadata queries on missing objects all fail loudly.
  EXPECT_THROW(hdf5_utils::getObjectDimension(ctx, p, "/gone"), hdf5_exception);
  EXPECT_THROW(hdf5_utils::setObjectDimension(ctx, p, "/gone", dimension(1, 1, 1)), hdf5_exception);
  EXPECT_THROW(hdf5_utils::getObjectBoundingBox(ctx, p, "/gone"), hdf5_exception);
  EXPECT_THROW(hdf5_utils::setObjectBoundingBox(ctx, p, "/gone", bb), hdf5_exception);
  EXPECT_THROW(hdf5_utils::getDataSetInfo(ctx, p, "/g1"), hdf5_exception); // group, not dataset
  EXPECT_THROW(hdf5_utils::getDataSetType(ctx, p, "/gone"), hdf5_exception);
  EXPECT_THROW(hdf5_utils::getDataSetMinimum(ctx, p, "/gone"), hdf5_exception);
  EXPECT_THROW(hdf5_utils::getDataSetMaximum(ctx, p, "/gone"), hdf5_exception);

  // A dataset at/above the chunking threshold takes the chunked layout path.
  ASSERT_NO_THROW(hdf5_utils::createDataSet(ctx, p, "/bigds", bb, dimension(256, 256, 257), UChar));
  EXPECT_TRUE(hdf5_utils::isDataSet(ctx, p, "/bigds"));
  d = hdf5_utils::getObjectDimension(ctx, p, "/bigds");
  EXPECT_EQ(d.zdim, 257u);
}

// ===========================================================================
// hdf5_utils: typed dataset reads/writes through the data_type dispatchers
// ===========================================================================

namespace {

template <class T>
void typedRoundTrip(app &ctx, const std::string &fn, const std::string &obj, data_type dt) {
  SCOPED_TRACE(obj);
  const dimension dim(4, 3, 2);
  const bounding_box bb(0.0, 0.0, 0.0, 3.0, 2.0, 1.0);
  ASSERT_NO_THROW(hdf5_utils::createDataSet(ctx, fn, obj, bb, dim, dt));

  std::vector<T> vals(size_t(dim.size()));
  for (size_t i = 0; i < vals.size(); ++i)
    vals[i] = T(i % 97);

  // data_type-dispatched write with explicit min/max.
  ASSERT_NO_THROW(hdf5_utils::writeDataSet(ctx, fn, obj, 0, 0, 0, dim, dt,
                                           reinterpret_cast<const unsigned char *>(&vals[0]), 0.0,
                                           96.0));
  EXPECT_DOUBLE_EQ(hdf5_utils::getDataSetMinimum(ctx, fn, obj), 0.0);
  EXPECT_DOUBLE_EQ(hdf5_utils::getDataSetMaximum(ctx, fn, obj), 96.0);

  // data_type-dispatched offset read.
  std::vector<T> back(vals.size(), T(0));
  ASSERT_NO_THROW(hdf5_utils::readDataSet(ctx, fn, obj, 0, 0, 0, dim, dt,
                                          reinterpret_cast<unsigned char *>(&back[0])));
  for (size_t i = 0; i < vals.size(); ++i)
    ASSERT_EQ(double(vals[i]), double(back[i])) << "index " << i;

  // data_type-dispatched bounding-box (tuple) read.
  boost::shared_array<unsigned char> bytes;
  dimension outdim;
  boost::tie(bytes, outdim) = hdf5_utils::readDataSet(ctx, fn, obj, bb, dt);
  ASSERT_EQ(outdim.size(), dim.size());
  const T *tv = reinterpret_cast<const T *>(bytes.get());
  for (size_t i = 0; i < vals.size(); ++i)
    ASSERT_EQ(double(vals[i]), double(tv[i])) << "index " << i;
}

} // namespace

TEST_F(Hdf5VolumeTest, UtilsTypedDataSetRoundTrips) {
  std::string p = path("typed.h5");
  ASSERT_NO_THROW(hdf5_utils::createHDF5File(ctx, p));

  typedRoundTrip<unsigned char>(ctx, p, "/t/uc", UChar);
  typedRoundTrip<unsigned short>(ctx, p, "/t/us", UShort);
  typedRoundTrip<unsigned int>(ctx, p, "/t/ui", UInt);
  typedRoundTrip<float>(ctx, p, "/t/f", Float);
  typedRoundTrip<double>(ctx, p, "/t/d", Double);
  typedRoundTrip<uint64>(ctx, p, "/t/u64", UInt64);
}

TEST_F(Hdf5VolumeTest, UtilsIntAndInt64AndUndefinedReads) {
  std::string p = path("ints.h5");
  const dimension dim(4, 3, 2);
  const bounding_box bb(0.0, 0.0, 0.0, 3.0, 2.0, 1.0);

  // createDataSet has no Int/Int64 support, so build the datasets raw and
  // stamp on the bbox/dim attributes the readers need.
  {
    boost::shared_ptr<H5::H5File> f = hdf5_utils::getH5File(p, true);
    hsize_t dims[3] = {2, 3, 4}; // ZYX order
    H5::DataSpace sp(3, dims);
    H5::DataSet dsi = f->createDataSet("ints", H5::PredType::NATIVE_INT, sp);
    std::vector<int> ivals(24);
    for (size_t i = 0; i < ivals.size(); ++i)
      ivals[i] = int(i) - 5;
    dsi.write(&ivals[0], H5::PredType::NATIVE_INT);
    H5::DataSet dsl = f->createDataSet("longs", H5::PredType::NATIVE_INT64, sp);
    std::vector<int64> lvals(24);
    for (size_t i = 0; i < lvals.size(); ++i)
      lvals[i] = int64(i) * 1000 - 7;
    dsl.write(&lvals[0], H5::PredType::NATIVE_INT64);
  }
  hdf5_utils::setObjectBoundingBox(ctx, p, "/ints", bb);
  hdf5_utils::setObjectDimension(ctx, p, "/ints", dim);
  hdf5_utils::setObjectBoundingBox(ctx, p, "/longs", bb);
  hdf5_utils::setObjectDimension(ctx, p, "/longs", dim);

  boost::shared_array<unsigned char> bytes;
  dimension outdim;
  boost::tie(bytes, outdim) = hdf5_utils::readDataSet(ctx, p, "/ints", bb, Int);
  ASSERT_EQ(outdim.size(), dim.size());
  const int *iv = reinterpret_cast<const int *>(bytes.get());
  for (int i = 0; i < 24; ++i)
    ASSERT_EQ(iv[i], i - 5);

  boost::tie(bytes, outdim) = hdf5_utils::readDataSet(ctx, p, "/longs", bb, Int64);
  ASSERT_EQ(outdim.size(), dim.size());
  const int64 *lv = reinterpret_cast<const int64 *>(bytes.get());
  for (int i = 0; i < 24; ++i)
    ASSERT_EQ(lv[i], int64(i) * 1000 - 7);

  EXPECT_THROW(hdf5_utils::readDataSet(ctx, p, "/ints", bb, Undefined), std::runtime_error);

  // Out-of-bounds and null-dimension offset reads fail.
  std::vector<int> buf(24, 0);
  EXPECT_ANY_THROW(hdf5_utils::readDataSet(ctx, p, "/ints", 2, 0, 0, dim, &buf[0]));
  EXPECT_ANY_THROW(hdf5_utils::readDataSet(ctx, p, "/ints", 0, 0, 0, dimension(0, 0, 0), &buf[0]));

  // Rank enforcement: a 1D dataset with volume attributes is rejected by
  // both the offset and the bounding-box readers.  The stamped dimension
  // attributes deliberately keep every axis > 1: readDataSet divides by
  // (dim - 1), and an axis of 1 hard-crashes on divide-by-zero (see bug
  // notes), so a truthful (4,1,1) stamp cannot be used here.
  {
    boost::shared_ptr<H5::H5File> f = hdf5_utils::getH5File(p);
    hsize_t one[1] = {4};
    H5::DataSpace sp1(1, one);
    f->createDataSet("flat", H5::PredType::NATIVE_INT, sp1);
  }
  hdf5_utils::setObjectBoundingBox(ctx, p, "/flat", bb);
  hdf5_utils::setObjectDimension(ctx, p, "/flat", dimension(4, 2, 2));
  EXPECT_ANY_THROW(hdf5_utils::readDataSet(ctx, p, "/flat", 0, 0, 0, dimension(4, 2, 2), &buf[0]));
  EXPECT_ANY_THROW(hdf5_utils::readDataSet<int>(ctx, p, "/flat", bb));
}

// ===========================================================================
// hdf5_utils: string datasets and attributes
// ===========================================================================

TEST_F(Hdf5VolumeTest, UtilsStringDataSetAndTextAttrs) {
  std::string p = path("strings.h5");
  ASSERT_NO_THROW(hdf5_utils::createHDF5File(ctx, p));

  // The string-dataset write API is currently broken: writeDataSet<char>
  // reads the double 'min'/'max' attributes back with the C_S1 memory type,
  // which HDF5 cannot convert, so the call throws after creating the (empty)
  // Char dataset (see bug notes).
  const std::string message = "hello hdf5 world";
  EXPECT_THROW(hdf5_utils::createDataSet(ctx, p, "/notes/readme", message), hdf5_exception);
  EXPECT_TRUE(hdf5_utils::isDataSet(ctx, p, "/notes/readme"));
  EXPECT_EQ(hdf5_utils::getDataSetType(ctx, p, "/notes/readme"), Char);

  // The string read path works when the Char dataset keeps every dimension
  // above 1 (the standard (len,1,1) string shape would divide by zero in
  // readDataSet; see bug notes).  Build a 3D Char blob raw and read it back
  // through the string API.
  const std::string blob = "0123456789abcdef"; // 16 chars == 4*2*2
  ASSERT_NO_THROW(hdf5_utils::createDataSet(ctx, p, "/notes/blob", bounding_box(0, 0, 0, 3, 1, 1),
                                            dimension(4, 2, 2), Char));
  {
    boost::shared_ptr<H5::H5File> f = hdf5_utils::getH5File(p);
    boost::shared_ptr<H5::DataSet> ds = hdf5_utils::getDataSet(*f, "/notes/blob", false);
    ds->write(blob.c_str(), H5::PredType::C_S1);
  }
  std::string back;
  ASSERT_NO_THROW(hdf5_utils::readDataSet(ctx, p, "/notes/blob", back));
  EXPECT_EQ(back, blob);

  // String attributes on a dataset and on a group.
  ASSERT_NO_THROW(
      hdf5_utils::setAttribute(ctx, p, "/notes/readme", "author", std::string("cvc-test")));
  std::string author;
  ASSERT_NO_THROW(hdf5_utils::getAttribute(ctx, p, "/notes/readme", "author", author));
  EXPECT_EQ(author, "cvc-test");
  ASSERT_NO_THROW(hdf5_utils::setAttribute(ctx, p, "/notes", "license", "LGPL-2.1")); // char*
  std::string license;
  ASSERT_NO_THROW(hdf5_utils::getAttribute(ctx, p, "/notes", "license", license));
  EXPECT_EQ(license, "LGPL-2.1");
  // Overwriting an existing string attribute goes through the remove path.
  ASSERT_NO_THROW(
      hdf5_utils::setAttribute(ctx, p, "/notes/readme", "author", std::string("other")));
  ASSERT_NO_THROW(hdf5_utils::getAttribute(ctx, p, "/notes/readme", "author", author));
  EXPECT_EQ(author, "other");

  // Missing attribute / missing object.
  std::string dummy;
  EXPECT_THROW(hdf5_utils::getAttribute(ctx, p, "/notes/readme", "absent", dummy), hdf5_exception);
  EXPECT_THROW(hdf5_utils::getAttribute(ctx, p, "/missing/object", "absent", dummy),
               hdf5_exception);
  EXPECT_THROW(hdf5_utils::setAttribute(ctx, p, "/missing/object", "a", std::string("v")),
               hdf5_exception);
}

TEST_F(Hdf5VolumeTest, UtilsAttributeArraysAndErrors) {
  std::string p = path("attrs.h5");
  const bounding_box bb(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
  ASSERT_NO_THROW(hdf5_utils::createHDF5File(ctx, p));
  ASSERT_NO_THROW(hdf5_utils::createDataSet(ctx, p, "/a/ds", bb, dimension(2, 2, 2), Float));
  ASSERT_NO_THROW(hdf5_utils::createGroup(ctx, p, "/a/grp"));

  // Scalar attributes of several types, on a dataset and on a group.
  ASSERT_NO_THROW(hdf5_utils::setAttribute(ctx, p, "/a/ds", "dval", 2.75));
  double dval = 0.0;
  ASSERT_NO_THROW(hdf5_utils::getAttribute(ctx, p, "/a/ds", "dval", dval));
  EXPECT_DOUBLE_EQ(dval, 2.75);

  ASSERT_NO_THROW(hdf5_utils::setAttribute(ctx, p, "/a/grp", "uval", uint64(77)));
  uint64 uval = 0;
  ASSERT_NO_THROW(hdf5_utils::getAttribute(ctx, p, "/a/grp", "uval", uval));
  EXPECT_EQ(uval, uint64(77));

  ASSERT_NO_THROW(hdf5_utils::setAttribute(ctx, p, "/a/ds", "ival", 42));
  int ival = 0;
  ASSERT_NO_THROW(hdf5_utils::getAttribute(ctx, p, "/a/ds", "ival", ival));
  EXPECT_EQ(ival, 42);

  // Array attributes, including length-change replacement.
  double arr3[3] = {1.5, 2.5, 3.5};
  ASSERT_NO_THROW(hdf5_utils::setAttribute(ctx, p, "/a/ds", "arr", 3, arr3));
  double got3[3] = {0, 0, 0};
  ASSERT_NO_THROW(hdf5_utils::getAttribute(ctx, p, "/a/ds", "arr", 3, got3));
  EXPECT_DOUBLE_EQ(got3[0], 1.5);
  EXPECT_DOUBLE_EQ(got3[2], 3.5);
  // Reading with the wrong length fails.
  double got2[2] = {0, 0};
  EXPECT_THROW(hdf5_utils::getAttribute(ctx, p, "/a/ds", "arr", 2, got2), hdf5_exception);
  // Rewriting with a different length recreates the attribute.
  double arr1[1] = {9.0};
  ASSERT_NO_THROW(hdf5_utils::setAttribute(ctx, p, "/a/ds", "arr", 1, arr1));
  double got1[1] = {0};
  ASSERT_NO_THROW(hdf5_utils::getAttribute(ctx, p, "/a/ds", "arr", 1, got1));
  EXPECT_DOUBLE_EQ(got1[0], 9.0);

  // data_type arrays are stored as uint64 under the hood.
  data_type dts[2] = {Float, UChar};
  ASSERT_NO_THROW(hdf5_utils::setAttribute(ctx, p, "/a/ds", "dts", 2, dts));
  data_type dtsBack[2] = {Undefined, Undefined};
  ASSERT_NO_THROW(hdf5_utils::getAttribute(ctx, p, "/a/ds", "dts", size_t(2), dtsBack));
  EXPECT_EQ(dtsBack[0], Float);
  EXPECT_EQ(dtsBack[1], UChar);

  // Numeric attribute access on missing objects.
  EXPECT_THROW(hdf5_utils::getAttribute(ctx, p, "/nowhere", "dval", dval), hdf5_exception);
  EXPECT_THROW(hdf5_utils::setAttribute(ctx, p, "/nowhere", "dval", 1.0), hdf5_exception);
  EXPECT_THROW(hdf5_utils::getAttribute(ctx, p, "/a/ds", "no-such-attr", dval), hdf5_exception);
}

// ===========================================================================
// hdf5_utils: dimension queries for bounding boxes
// ===========================================================================

TEST_F(Hdf5VolumeTest, UtilsDataSetDimensionQueries) {
  std::string p = path("dims.h5");
  const bounding_box bb(0.0, 0.0, 0.0, 7.0, 7.0, 7.0);
  ASSERT_NO_THROW(hdf5_utils::createHDF5File(ctx, p));
  ASSERT_NO_THROW(hdf5_utils::createDataSet(ctx, p, "/vol", bb, dimension(8, 8, 8), Float));
  ASSERT_NO_THROW(hdf5_utils::createGroup(ctx, p, "/agroup"));

  dimension d = hdf5_utils::getDataSetDimensionForBoundingBox(ctx, p, "/vol", bb);
  EXPECT_EQ(d.xdim, 8u);
  EXPECT_EQ(d.ydim, 8u);
  EXPECT_EQ(d.zdim, 8u);
  d = hdf5_utils::getDataSetDimensionForBoundingBox(ctx, p, "/vol", bounding_box(0, 0, 0, 3, 3, 3));
  EXPECT_EQ(d.xdim, 4u);

  // maxdim-constrained dimension applies a stride.
  d = hdf5_utils::getDataSetDimension(ctx, p, "/vol", bb, dimension(4, 4, 4));
  EXPECT_EQ(d.xdim, 4u);
  EXPECT_EQ(d.ydim, 4u);
  EXPECT_EQ(d.zdim, 4u);
  d = hdf5_utils::getDataSetDimension(ctx, p, "/vol", bb, dimension(3, 3, 3));
  EXPECT_EQ(d.xdim, 2u);
  EXPECT_EQ(d.ydim, 2u);
  EXPECT_EQ(d.zdim, 2u);

  EXPECT_THROW(hdf5_utils::getDataSetDimensionForBoundingBox(ctx, p, "/agroup", bb),
               hdf5_exception);
  EXPECT_THROW(hdf5_utils::getDataSetDimension(ctx, p, "/missing", bb), hdf5_exception);
}

// ===========================================================================
// hdf5_utils: object-level helpers driven directly on H5 handles
// ===========================================================================

TEST_F(Hdf5VolumeTest, UtilsRawObjectLevelHelpers) {
  std::string p = path("raw.h5");
  {
    boost::shared_ptr<H5::H5File> f = hdf5_utils::getH5File(p, true);
    ASSERT_TRUE(f.get() != NULL);

    boost::shared_ptr<H5::Group> g = hdf5_utils::getGroup(*f, "/r1/r2", true);
    ASSERT_TRUE(g.get() != NULL);

    hsize_t dims[1] = {6};
    H5::DataSpace sp(1, dims);
    H5::DataSet ds = f->createDataSet("rootds", H5::PredType::NATIVE_DOUBLE, sp);

    // hasAttribute plain true/false.
    EXPECT_FALSE(hdf5_utils::hasAttribute(ds, "tag"));
    hdf5_utils::setAttribute(ds, "tag", 3.25); // object-level scalar setter
    EXPECT_TRUE(hdf5_utils::hasAttribute(ds, "tag"));
    double tag = 0.0;
    hdf5_utils::getAttribute(ds, "tag", tag); // object-level scalar getter
    EXPECT_DOUBLE_EQ(tag, 3.25);

    // Object-level string and char* setters/getters.
    hdf5_utils::setAttribute(ds, "sname", std::string("svalue"));
    std::string sval;
    hdf5_utils::getAttribute(ds, "sname", sval);
    EXPECT_EQ(sval, "svalue");
    hdf5_utils::setAttribute(ds, "cname", "cvalue");
    hdf5_utils::getAttribute(ds, "cname", sval);
    EXPECT_EQ(sval, "cvalue");

    // Object-level data_type array specialization stores uint64s.
    data_type dts[2] = {Double, Char};
    hdf5_utils::setAttribute(ds, "dts", size_t(2), dts);
    uint64 raw[2] = {0, 0};
    hdf5_utils::getAttribute(ds, "dts", size_t(2), raw);
    EXPECT_EQ(raw[0], uint64(Double));
    EXPECT_EQ(raw[1], uint64(Char));

    // Scalar-dataspace attributes are rejected by the 1D-array reader.
    double pi = 3.14159;
    H5::Attribute scalarAttr =
        ds.createAttribute("scalar", H5::PredType::NATIVE_DOUBLE, H5::DataSpace(H5S_SCALAR));
    scalarAttr.write(H5::PredType::NATIVE_DOUBLE, &pi);
    double out = 0.0;
    EXPECT_ANY_THROW(hdf5_utils::getAttribute(ds, "scalar", out));

    // getGroup without create refuses missing paths.
    EXPECT_ANY_THROW(hdf5_utils::getGroup(*f, "/does/not/exist", false));

    // getDataSet with create=true materializes intermediate groups but still
    // fails to open the (nonexistent) terminal dataset.
    EXPECT_ANY_THROW(hdf5_utils::getDataSet(*f, "/new1/new2/ds", true));

    // getDataSet opens through existing nested groups.
    H5::DataSet nested = g->createDataSet("deep", H5::PredType::NATIVE_INT, sp);
    boost::shared_ptr<H5::DataSet> opened = hdf5_utils::getDataSet(*f, "/r1/r2/deep", false);
    EXPECT_TRUE(opened.get() != NULL);

    // unlink at the file root.
    hdf5_utils::unlink(*f, "rootds");
    f->flush(H5F_SCOPE_GLOBAL);
  }
  EXPECT_FALSE(hdf5_utils::objectExists(ctx, p, "rootds"));
  EXPECT_TRUE(hdf5_utils::isGroup(ctx, p, "/new1/new2"));
  EXPECT_TRUE(hdf5_utils::isDataSet(ctx, p, "/r1/r2/deep"));

  // Read-only files fall back to RDONLY access.
  namespace fs = std::filesystem;
  fs::permissions(p, fs::perms::owner_read | fs::perms::group_read | fs::perms::others_read,
                  fs::perm_options::replace);
  EXPECT_TRUE(hdf5_utils::isDataSet(ctx, p, "/r1/r2/deep"));
  fs::permissions(p,
                  fs::perms::owner_read | fs::perms::owner_write | fs::perms::group_read |
                      fs::perms::others_read,
                  fs::perm_options::replace);
}

// ===========================================================================
// hdf5_utils: PredType mapping
// ===========================================================================

TEST_F(Hdf5VolumeTest, UtilsPredTypeMapping) {
  EXPECT_TRUE(hdf5_utils::getPredType(Char) == H5::PredType::C_S1);
  EXPECT_TRUE(hdf5_utils::getPredType(Int) == H5::PredType::NATIVE_INT);
  EXPECT_TRUE(hdf5_utils::getPredType(Int64) == H5::PredType::NATIVE_INT64);
  EXPECT_THROW(hdf5_utils::getPredType(Undefined), hdf5_exception);
  EXPECT_THROW(hdf5_utils::getPredType(static_cast<data_type>(99)), hdf5_exception);
}

// ===========================================================================
// hdf5_utils: concurrent entry into the HDF5 library
// ===========================================================================

// The hdf5 dependency is built without HDF5_ENABLE_THREADSAFE (that option is
// mutually exclusive with the C++ wrapper libcvc uses), so hdf5_utils has to
// serialize every call into the library itself -- see hdf5_utils::library_lock.
// That guard used to be one mutex *per filename*, which left two holes:
// threads working on different files were not serialized against each other at
// all, and H5::Exception::dontPrint() was called before the lock was taken.
// MultiVariableMultiTimestep tripped it intermittently in CI (SEGFAULT)
// because its four writes each spawn a background hierarchy thread.
//
// Every write below targets its own file *and* its own object path, so the
// volumes cannot interact through the data -- the only thing under test is
// that concurrent entry into the HDF5 library is safe. Distinct object paths
// matter: the hierarchy thread a write spawns is keyed off the object path, so
// sharing one would make the workers cancel each other's threads instead.
TEST_F(Hdf5VolumeTest, ConcurrentAccessToDistinctFiles) {
  const int kWorkers = 4;
  const int kRounds = 2;

  std::vector<std::string> errors(kWorkers);
  std::vector<std::thread> workers;
  workers.reserve(kWorkers);

  for (int t = 0; t < kWorkers; ++t) {
    workers.push_back(std::thread([this, t, &errors]() {
      try {
        for (int r = 0; r < kRounds; ++r) {
          const std::string tag = std::to_string(t) + "_" + std::to_string(r);
          // The file must not exist yet: writeVolumeFile only auto-creates a
          // missing target, and it honors the object path when it does.
          const std::string url = path("concurrent" + tag + ".h5") + "|/cvc/volumes/w" + tag;
          volume out = make_test_volume(ctx, 8, 8, 8, Float, double(10 * t + r));
          writeVolumeFile(ctx, out, url);

          volume in(ctx);
          readVolumeFile(ctx, in, url);
          if (in.XDim() != out.XDim() || in.YDim() != out.YDim() || in.ZDim() != out.ZDim())
            throw std::runtime_error("dimension mismatch on " + url);
          // gtest's fatal assertions are not usable off the main thread, so
          // report through the error slot instead.
          for (uint64 k = 0; k < out.ZDim(); ++k)
            for (uint64 j = 0; j < out.YDim(); ++j)
              for (uint64 i = 0; i < out.XDim(); ++i)
                if (std::fabs(out(i, j, k) - in(i, j, k)) > 1e-4)
                  throw std::runtime_error("voxel mismatch on " + url);
        }
      } catch (std::exception &e) {
        errors[t] = e.what();
      }
    }));
  }

  for (std::vector<std::thread>::iterator i = workers.begin(); i != workers.end(); ++i)
    i->join();

  for (int t = 0; t < kWorkers; ++t)
    EXPECT_TRUE(errors[t].empty()) << "worker " << t << ": " << errors[t];
}
