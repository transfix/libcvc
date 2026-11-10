/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The file handler tables belong to the app (VOLROVER-LIBRARY-PORTS-ROADMAP.md 1.3, P0a's file
// I/O step). Before, every cvc::app constructor added a fresh set of handlers to one process-wide
// volume map and one geometry map, so each app made added duplicates every other app then saw,
// and a handler one app registered was every app's.
//
//   two apps:  each sees only its own handlers, of all four kinds;
//   making an app (on this thread or others, or app(no_init)) changes no other app's tables;
//   order:     reads give the same answers in any order and on any app, and making apps between
//              them changes nothing.

#include <algorithm>
#include <atomic>
#include <boost/shared_ptr.hpp>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/file_handler_registry.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/image/image.h>
#include <cvc/model/model_file_io.h>
#include <cvc/volume/io_handlers.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_info.h>
#include <cvc/volume/volume_file_io.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <map>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

namespace {

// ── one fake handler of each kind, serving the made-up extension ".twoapps" ──

struct fake_volume_io : cvc::volume_file_io {
  explicit fake_volume_io(std::string id) : _id(std::move(id)) { _ext.push_back(".twoapps"); }
  const std::string &id() const override { return _id; }
  const extension_list &extensions() const override { return _ext; }
  void getVolumeFileInfo(cvc::app &, cvc::volume_file_info::data &,
                         const std::string &) const override {}
  void readVolumeFile(cvc::app &, cvc::volume &vol, const std::string &, unsigned int, unsigned int,
                      cvc::uint64, cvc::uint64, cvc::uint64,
                      const cvc::dimension &) const override {
    ++reads;
    vol.resize(cvc::dimension(3, 3, 3));
  }
  void createVolumeFile(cvc::app &, const std::string &, const cvc::bounding_box &,
                        const cvc::dimension &, const std::vector<cvc::data_type> &, unsigned int,
                        unsigned int, double, double) const override {}
  void writeVolumeFile(cvc::app &, const cvc::volume &, const std::string &, unsigned int,
                       unsigned int, cvc::uint64, cvc::uint64, cvc::uint64) const override {}
  mutable int reads = 0;
  std::string _id;
  extension_list _ext;
};

struct fake_geometry_io : cvc::geometry_file_io {
  fake_geometry_io() : _id("fake_geometry_io") { _ext.push_back(".twoapps"); }
  const std::string &id() const override { return _id; }
  const extension_list &extensions() const override { return _ext; }
  cvc::geometry read(cvc::app &ctx, const std::string &) const override {
    cvc::geometry g(ctx);
    g.points().push_back({{1.0, 2.0, 3.0}});
    return g;
  }
  void write(cvc::app &, const cvc::geometry &, const std::string &) const override {}
  std::string _id;
  extension_list _ext;
};

struct fake_model_io : cvc::model_file_io {
  fake_model_io() : _id("fake_model_io") { _ext.push_back(".twoapps"); }
  const std::string &id() const override { return _id; }
  const extension_list &extensions() const override { return _ext; }
  cvc::model read(cvc::app &, const std::string &) const override {
    cvc::model m;
    m.meshes.resize(2);
    return m;
  }
  void write(cvc::app &, const cvc::model &, const std::string &) const override {}
  std::string _id;
  extension_list _ext;
};

struct fake_image_io : cvc::image_file_io {
  fake_image_io() : _ext{"twoapps"} {}
  std::string id() const override { return "fake_image_io"; }
  const std::list<std::string> &extensions() const override { return _ext; }
  bool can_read(const std::string &path) const override {
    return cvc::image_file_extension(path) == "twoapps";
  }
  cvc::image read(const std::string &) const override { return cvc::image(7, 5); }
  void write(const cvc::image &, const std::string &, int) const override {}
  std::list<std::string> _ext;
};

// Every handler id in each of an app's four tables, with how many times it is registered.
std::map<std::string, int> census(cvc::app &a) {
  std::map<std::string, int> ids;
  for (const auto &h : a.volume_handlers().all())
    ++ids["volume:" + h->id()];
  for (const auto &h : a.geometry_handlers().all())
    ++ids["geometry:" + h->id()];
  for (const auto &h : a.model_handlers().all())
    ++ids["model:" + h->id()];
  for (const auto &h : a.image_handlers().all())
    ++ids["image:" + h->id()];
  return ids;
}

std::string temp_path(const std::string &name) {
  return (fs::temp_directory_path() / ("cvc_file_io_registry_" + name)).string();
}

} // namespace

TEST(FileIoRegistry, EveryAppStartsWithTheBuiltInHandlersOnce) {
  cvc::app a;
  EXPECT_FALSE(a.volume_handlers().empty());
  EXPECT_FALSE(a.geometry_handlers().empty());
  EXPECT_FALSE(a.image_handlers().empty());
  for (const auto &[id, n] : census(a))
    EXPECT_EQ(n, 1) << id << " is registered " << n << " times";
  EXPECT_FALSE(a.volume_handlers().for_extension(".rawiv").empty());
  EXPECT_FALSE(a.geometry_handlers().for_extension(".off").empty());
  EXPECT_FALSE(a.geometry_handlers().for_extension(".bunny").empty());
}

TEST(FileIoRegistry, TwoAppsSeeOnlyTheirOwnHandlers) {
  cvc::app a, b;
  const auto before = census(b);

  auto vol = boost::shared_ptr<fake_volume_io>(new fake_volume_io("fake_volume_io"));
  ASSERT_TRUE(a.volume_handlers().insert(vol));
  ASSERT_TRUE(a.geometry_handlers().insert(cvc::geometry_file_io::ptr(new fake_geometry_io)));
  ASSERT_TRUE(a.model_handlers().insert(cvc::model_file_io::ptr(new fake_model_io)));
  ASSERT_TRUE(a.image_handlers().insert(cvc::image_file_io::ptr(new fake_image_io)));

  // b's tables are as they were, and none of its lookups finds a's handlers.
  EXPECT_EQ(census(b), before);
  EXPECT_TRUE(b.volume_handlers().for_extension(".twoapps").empty());
  EXPECT_TRUE(b.geometry_handlers().for_extension(".twoapps").empty());
  EXPECT_TRUE(b.model_handlers().for_extension(".twoapps").empty());
  EXPECT_FALSE(cvc::image_file_io::for_read(b, "x.twoapps"));

  // a reads the made-up format with its handlers; b cannot.
  cvc::volume va(a), vb(b);
  cvc::readVolumeFile(a, va, "x.twoapps");
  EXPECT_EQ(vol->reads, 1);
  EXPECT_EQ(va.XDim(), 3u);
  EXPECT_THROW(cvc::readVolumeFile(b, vb, "x.twoapps"), cvc::unsupported_volume_file_type);
  EXPECT_EQ(vol->reads, 1);

  EXPECT_EQ(cvc::read_geometry(a, "x.twoapps").num_points(), 1u);
  EXPECT_THROW(cvc::read_geometry(b, "x.twoapps"), cvc::unsupported_geometry_file_type);

  EXPECT_EQ(cvc::read_model(a, "x.twoapps").meshes.size(), 2u);
  EXPECT_THROW(cvc::read_model(b, "x.twoapps"), cvc::unsupported_model_file_type);

  EXPECT_EQ(cvc::read_image(a, "x.twoapps").width(), 7);
  EXPECT_ANY_THROW(cvc::read_image(b, "x.twoapps"));

  // What b knows is its own business too: removing a handler from b leaves a's.
  b.volume_handlers().remove(std::string("rawiv_io : v1.0"));
  EXPECT_TRUE(b.volume_handlers().for_extension(".rawiv").empty());
  EXPECT_FALSE(a.volume_handlers().for_extension(".rawiv").empty());

  // And a can drop its own again.
  a.volume_handlers().remove(vol);
  cvc::volume again(a);
  EXPECT_THROW(cvc::readVolumeFile(a, again, "x.twoapps"), cvc::unsupported_volume_file_type);
}

TEST(FileIoRegistry, MakingAnAppChangesNoOtherApp) {
  cvc::app a;
  const auto before = census(a);
  {
    cvc::app b, c;
    cvc::app bare{cvc::no_init_t{}};
    EXPECT_EQ(census(a), before);
    EXPECT_EQ(census(b), before); // every app starts alike
    EXPECT_EQ(census(c), before);
    EXPECT_TRUE(census(bare).empty()); // app(no_init) starts with none
  }
  EXPECT_EQ(census(a), before); // nor does an app going away

  // Apps made on other threads at once (as the state volume codec makes one per thread) leave a
  // as it was, and each has one of each handler.
  std::atomic<int> dupes{0}, differ{0};
  std::vector<std::thread> threads;
  for (int t = 0; t < 4; ++t)
    threads.emplace_back([&] {
      cvc::app mine;
      const auto ids = census(mine);
      if (ids != before)
        ++differ;
      for (const auto &kv : ids)
        if (kv.second != 1)
          ++dupes;
    });
  for (auto &t : threads)
    t.join();
  EXPECT_EQ(dupes.load(), 0);
  EXPECT_EQ(differ.load(), 0);
  EXPECT_EQ(census(a), before);
}

TEST(FileIoRegistry, NoInitAppGetsTheDefaultsWhenAsked) {
  cvc::app bare{cvc::no_init_t{}};
  cvc::volume v(bare);
  EXPECT_THROW(cvc::readVolumeFile(bare, v, "x.rawiv"), cvc::unsupported_volume_file_type);
  EXPECT_THROW(cvc::read_geometry(bare, "x.bunny"), cvc::unsupported_geometry_file_type);
  cvc::register_default_volume_handlers(bare);
  cvc::register_default_geometry_handlers(bare);
  EXPECT_EQ(cvc::read_geometry(bare, "x.bunny").num_tris(), 69473u);
  cvc::app full;
  const auto all = census(full);
  for (const auto &[id, n] : census(bare)) {
    EXPECT_EQ(n, 1) << id;
    EXPECT_EQ(all.count(id), 1u) << id;
  }
}

TEST(FileIoRegistry, ATableHoldsOneHandlerPerId) {
  cvc::app a;
  const std::size_t n = a.volume_handlers().size();
  // A second object with an id already there is not added; neither is the same object again.
  auto first = boost::shared_ptr<fake_volume_io>(new fake_volume_io("dup_io"));
  auto second = boost::shared_ptr<fake_volume_io>(new fake_volume_io("dup_io"));
  EXPECT_TRUE(a.volume_handlers().insert(first));
  EXPECT_FALSE(a.volume_handlers().insert(first));
  EXPECT_FALSE(a.volume_handlers().insert(second));
  EXPECT_FALSE(a.volume_handlers().insert(cvc::volume_file_io::ptr()));
  EXPECT_EQ(a.volume_handlers().size(), n + 1);
  const auto ext = a.volume_handlers().extensions();
  EXPECT_TRUE(std::find(ext.begin(), ext.end(), ".twoapps") != ext.end());
  EXPECT_EQ(a.volume_handlers().by_extension().at(".twoapps").size(), 1u);
  a.volume_handlers().remove(std::string("dup_io"));
  EXPECT_EQ(a.volume_handlers().size(), n);
}

// Order: the same reads give the same answers whatever ran before them, on whichever app, with
// apps made and destroyed between them (the old tables grew with every app made).
TEST(FileIoRegistry, ReadsGiveTheSameAnswersInAnyOrder) {
  const std::string vol_path = temp_path("order.rawiv");
  const std::string geo_path = temp_path("order.off");
  {
    cvc::app writer;
    cvc::volume v(writer, cvc::dimension(5, 4, 3), cvc::Float);
    for (cvc::uint64 i = 0; i < v.XDim() * v.YDim() * v.ZDim(); ++i)
      v(i, static_cast<double>(i % 7));
    cvc::writeVolumeFile(writer, v, vol_path);
    cvc::geometry g(writer);
    g.points().push_back({{0.0, 0.0, 0.0}});
    g.points().push_back({{1.0, 0.0, 0.0}});
    g.points().push_back({{0.0, 1.0, 0.0}});
    g.tris().push_back({{0, 1, 2}});
    cvc::write_geometry(writer, g, geo_path);
  }

  struct result {
    unsigned x = 0, y = 0, z = 0;
    double max = 0.0;
    std::size_t points = 0, tris = 0;
    bool operator==(const result &o) const {
      return x == o.x && y == o.y && z == o.z && max == o.max && points == o.points &&
             tris == o.tris;
    }
  };
  const auto read_vol = [&](cvc::app &a, result &r) {
    cvc::volume v(a);
    cvc::readVolumeFile(a, v, vol_path);
    r.x = static_cast<unsigned>(v.XDim());
    r.y = static_cast<unsigned>(v.YDim());
    r.z = static_cast<unsigned>(v.ZDim());
    r.max = v.max();
  };
  const auto read_geo = [&](cvc::app &a, result &r) {
    const cvc::geometry g = cvc::read_geometry(a, geo_path);
    r.points = g.num_points();
    r.tris = g.num_tris();
  };

  result alone;
  {
    cvc::app a;
    read_vol(a, alone);
  }
  {
    cvc::app a;
    read_geo(a, alone);
  }
  result ab, ba, interleaved;
  {
    cvc::app a;
    read_vol(a, ab);
    read_geo(a, ab);
  }
  {
    cvc::app a;
    read_geo(a, ba);
    read_vol(a, ba);
  }
  {
    cvc::app a;
    read_vol(a, interleaved);
    { cvc::app other, another; } // made and gone between the reads
    read_geo(a, interleaved);
  }
  EXPECT_EQ(alone.x, 5u);
  EXPECT_EQ(alone.points, 3u);
  EXPECT_TRUE(ab == alone);
  EXPECT_TRUE(ba == alone);
  EXPECT_TRUE(interleaved == alone);
  std::remove(vol_path.c_str());
  std::remove(geo_path.c_str());
}

TEST(FileIoRegistry, AGeometryWithoutAnAppHasNoHandlersToUse) {
  cvc::geometry g;
  EXPECT_THROW(g.read("x.bunny"), cvc::read_error);
  EXPECT_THROW(g.write(temp_path("nowhere.off")), cvc::write_error);
  cvc::app a;
  cvc::geometry h(a);
  h.read("x.bunny");
  EXPECT_EQ(h.num_tris(), 69473u);
}
