/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Axis-aligned slices (cvc/volume/volume_slice.h): the data side of the `volume_slice` scene node,
// the VolumeRover ports' P7 rows for it (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.7, §6):
//   - T2: axis names, the planes' axes (VolumeGridRover's XY / XZ / ZY), the grid's planes and
//     voxels, a slice's values down each axis, its colours through a transfer function;
//   - T5: a slice outside the volume, an empty volume, a missing or garbage file, a variable the
//     file lacks, NaN voxels;
//   - T9: a file read one plane at a time (region reads) gives exactly the planes of the volume
//   read
//     whole, down each axis, and the cache spares a re-read;
//   - T6: two apps read their own files; two threads cutting different slices get what each gets
//     alone;
//   - T8: volume_slice_params on the state tree.
// Runs natively and under node (the ports lane, T14).

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/core/param_table.h>
#include <cvc/state/state.h>
#include <cvc/tf/model.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
#include <cvc/volume/volume_slice.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace fs = std::filesystem;
using steady = std::chrono::steady_clock;

struct scratch_dir {
  fs::path path;
  explicit scratch_dir(const std::string &name) {
    path = fs::temp_directory_path() /
           ("cvc_slices_" + name + "_" + std::to_string(steady::now().time_since_epoch().count()));
    fs::create_directories(path);
  }
  ~scratch_dir() {
    std::error_code ec;
    fs::remove_all(path, ec);
  }
  std::string file(const std::string &name) const { return (path / name).generic_string(); }
};

// A float volume whose voxel (i, j, k) holds i + 100 j + 10000 k, over a box that is not the
// default, so a slice's values and positions both say where it came from.
cvc::volume coded(cvc::app &app, std::uint64_t nx = 5, std::uint64_t ny = 4, std::uint64_t nz = 3) {
  cvc::volume v(app, cvc::dimension(nx, ny, nz), cvc::Float,
                cvc::bounding_box(-2.0, 10.0, 0.0, 2.0, 13.0, 4.0));
  for (std::uint64_t k = 0; k < nz; ++k)
    for (std::uint64_t j = 0; j < ny; ++j)
      for (std::uint64_t i = 0; i < nx; ++i)
        v(i, j, k, static_cast<double>(i + 100 * j + 10000 * k));
  return v;
}

double code(std::uint64_t i, std::uint64_t j, std::uint64_t k) {
  return static_cast<double>(i + 100 * j + 10000 * k);
}

cvc::tf::model grayscale_opaque() {
  cvc::tf::model m = cvc::tf::model::preset("grayscale");
  m.alpha = {{0.0, 1.0f}, {1.0, 1.0f}};
  m.normalize();
  return m;
}

} // namespace

// ---- T2: names and planes ----

TEST(VolumeSlice, AxesByNameAndByView) {
  cvc::slice_axis a = cvc::slice_axis::z;
  struct row {
    const char *text;
    cvc::slice_axis axis;
  };
  for (const row &r :
       {row{"x", cvc::slice_axis::x}, row{" Y ", cvc::slice_axis::y}, row{"z", cvc::slice_axis::z},
        row{"0", cvc::slice_axis::x}, row{"2", cvc::slice_axis::z}, row{"xy", cvc::slice_axis::z},
        row{"XZ", cvc::slice_axis::y}, row{"zx", cvc::slice_axis::y}, row{"zy", cvc::slice_axis::x},
        row{"yz", cvc::slice_axis::x}}) {
    a = cvc::slice_axis::z;
    EXPECT_TRUE(cvc::parse_slice_axis(r.text, a)) << r.text;
    EXPECT_EQ(a, r.axis) << r.text;
  }
  EXPECT_FALSE(cvc::parse_slice_axis("w", a));
  EXPECT_FALSE(cvc::parse_slice_axis("", a));
  EXPECT_STREQ(cvc::slice_axis_name(cvc::slice_axis::y), "y");
  EXPECT_STREQ(cvc::slice_view_name(cvc::slice_axis::x), "zy");
  EXPECT_STREQ(cvc::slice_view_name(cvc::slice_axis::y), "xz");
  EXPECT_STREQ(cvc::slice_view_name(cvc::slice_axis::z), "xy");
  // The legacy views: XY (x right, y up), XZ (x right, z up), ZY (z right, y up).
  int r = -1, u = -1;
  cvc::slice_plane_axes(cvc::slice_axis::z, r, u);
  EXPECT_EQ(r, 0);
  EXPECT_EQ(u, 1);
  cvc::slice_plane_axes(cvc::slice_axis::y, r, u);
  EXPECT_EQ(r, 0);
  EXPECT_EQ(u, 2);
  cvc::slice_plane_axes(cvc::slice_axis::x, r, u);
  EXPECT_EQ(r, 2);
  EXPECT_EQ(u, 1);
}

TEST(VolumeSlice, TheGridsPlanesAndVoxels) {
  cvc::app app;
  const cvc::slice_grid g = cvc::grid_of(coded(app));
  EXPECT_EQ(g.count(cvc::slice_axis::x), 5u);
  EXPECT_EQ(g.count(cvc::slice_axis::z), 3u);
  EXPECT_DOUBLE_EQ(g.spacing(0), 1.0); // 4 units over 4 gaps
  EXPECT_DOUBLE_EQ(g.spacing(1), 1.0);
  EXPECT_DOUBLE_EQ(g.spacing(2), 2.0);
  EXPECT_DOUBLE_EQ(g.plane(cvc::slice_axis::z, 0), 0.0);
  EXPECT_DOUBLE_EQ(g.plane(cvc::slice_axis::z, 2), 4.0);
  EXPECT_DOUBLE_EQ(g.plane(cvc::slice_axis::z, 99), 4.0) << "clamped to the grid";
  EXPECT_DOUBLE_EQ(g.plane(cvc::slice_axis::y, 1), 11.0);
  EXPECT_EQ(g.nearest_index(cvc::slice_axis::z, 2.9), 1u);
  EXPECT_EQ(g.nearest_index(cvc::slice_axis::z, 3.1), 2u);
  EXPECT_EQ(g.nearest_index(cvc::slice_axis::x, -50.0), 0u);
  EXPECT_EQ(g.nearest_index(cvc::slice_axis::x, 50.0), 4u);
  const double p[3] = {0.4, 12.6, 1.2};
  std::int64_t ijk[3];
  EXPECT_TRUE(g.voxel_of(p, ijk));
  EXPECT_EQ(ijk[0], 2);
  EXPECT_EQ(ijk[1], 3);
  EXPECT_EQ(ijk[2], 1);
  double c[3];
  g.voxel_center(ijk, c);
  EXPECT_DOUBLE_EQ(c[0], 0.0);
  EXPECT_DOUBLE_EQ(c[1], 13.0);
  EXPECT_DOUBLE_EQ(c[2], 2.0);
  const double out[3] = {2.6, 10.0, 0.0};
  EXPECT_FALSE(g.voxel_of(out, ijk)) << "past the box by more than half a voxel";
  EXPECT_EQ(ijk[0], 5);
  const double nan[3] = {std::nan(""), 10.0, 0.0};
  EXPECT_FALSE(g.voxel_of(nan, ijk));
  // One voxel along an axis: drawn the box's extent there, or 1 when it has none.
  cvc::slice_grid flat{cvc::dimension(4, 4, 1), cvc::bounding_box(0, 0, 0, 3, 3, 0)};
  EXPECT_DOUBLE_EQ(flat.spacing(2), 1.0);
  EXPECT_EQ(flat.nearest_index(cvc::slice_axis::z, 7.0), 0u);
}

TEST(VolumeSlice, SlicesDownEachAxis) {
  cvc::app app;
  const cvc::volume v = coded(app);
  // Down z: x right, y up.
  const cvc::volume_slice_data z = cvc::extract_slice(v, cvc::slice_axis::z, 2);
  EXPECT_EQ(z.width, 5u);
  EXPECT_EQ(z.height, 4u);
  EXPECT_EQ(z.at(0, 0), code(0, 0, 2));
  EXPECT_EQ(z.at(4, 0), code(4, 0, 2));
  EXPECT_EQ(z.at(1, 3), code(1, 3, 2));
  // Down y: x right, z up.
  const cvc::volume_slice_data y = cvc::extract_slice(v, cvc::slice_axis::y, 1);
  EXPECT_EQ(y.width, 5u);
  EXPECT_EQ(y.height, 3u);
  EXPECT_EQ(y.at(3, 2), code(3, 1, 2));
  // Down x: z right, y up.
  const cvc::volume_slice_data x = cvc::extract_slice(v, cvc::slice_axis::x, 4);
  EXPECT_EQ(x.width, 3u);
  EXPECT_EQ(x.height, 4u);
  EXPECT_EQ(x.at(2, 1), code(4, 1, 2));
  std::uint64_t ijk[3];
  x.voxel(2, 1, ijk);
  EXPECT_EQ(ijk[0], 4u);
  EXPECT_EQ(ijk[1], 1u);
  EXPECT_EQ(ijk[2], 2u);
  // Every value of every slice, against the volume.
  for (int a = 0; a < 3; ++a) {
    const auto axis = static_cast<cvc::slice_axis>(a);
    const std::uint64_t n = cvc::grid_of(v).count(a);
    for (std::uint64_t s = 0; s < n; ++s) {
      const cvc::volume_slice_data d = cvc::extract_slice(v, axis, s);
      for (std::uint64_t r = 0; r < d.height; ++r)
        for (std::uint64_t c = 0; c < d.width; ++c) {
          d.voxel(c, r, ijk);
          ASSERT_EQ(d.at(c, r), v(ijk[0], ijk[1], ijk[2])) << "axis " << a << " slice " << s;
        }
    }
  }
}

TEST(VolumeSlice, ColoursThroughATransferFunction) {
  cvc::app app;
  cvc::volume v(app, cvc::dimension(3, 2, 1), cvc::Float);
  // Row 0 (the bottom): 0, 5, 10; row 1 (the top): 10, NaN, 2.5.
  const double vals[2][3] = {{0.0, 5.0, 10.0}, {10.0, std::nan(""), 2.5}};
  for (int j = 0; j < 2; ++j)
    for (int i = 0; i < 3; ++i)
      v(i, j, 0, vals[j][i]);
  const cvc::volume_slice_data s = cvc::extract_slice(v, cvc::slice_axis::z, 0);
  const cvc::tf::model gray = grayscale_opaque();
  const std::vector<std::uint8_t> px = cvc::color_slice(s, gray, 0.0, 10.0, true);
  ASSERT_EQ(px.size(), 3u * 2u * 4u);
  auto pixel = [&](const std::vector<std::uint8_t> &p, int x, int y) {
    const std::size_t at = (static_cast<std::size_t>(y) * 3 + x) * 4;
    return std::vector<int>{p[at], p[at + 1], p[at + 2], p[at + 3]};
  };
  // Image order: the slice's top row first.
  EXPECT_EQ(pixel(px, 0, 0), (std::vector<int>{255, 255, 255, 255})); // 10
  EXPECT_EQ(pixel(px, 1, 0), (std::vector<int>{0, 0, 0, 0}));         // NaN: transparent black
  EXPECT_NEAR(pixel(px, 2, 0)[0], 64, 1);                             // 2.5
  EXPECT_EQ(pixel(px, 0, 1), (std::vector<int>{0, 0, 0, 255}));       // 0
  EXPECT_NEAR(pixel(px, 1, 1)[0], 128, 1);                            // 5
  // Not opaque: the opacity comes from the model (the grayscale preset's ramp 0, .5, 1).
  const cvc::tf::model ramp = cvc::tf::model::preset("grayscale");
  const std::vector<std::uint8_t> tr = cvc::color_slice(s, ramp, 0.0, 10.0, false);
  EXPECT_EQ(pixel(tr, 0, 1)[3], 0);
  EXPECT_NEAR(pixel(tr, 1, 1)[3], 128, 1);
  EXPECT_EQ(pixel(tr, 0, 0)[3], 255);
  // An explicit domain wins over the data range: [0, 20] puts 10 half way.
  cvc::tf::model fixed = gray;
  fixed.domain.automatic = false;
  fixed.domain.min = 0.0;
  fixed.domain.max = 20.0;
  EXPECT_NEAR(pixel(cvc::color_slice(s, fixed, 0.0, 10.0, true), 0, 0)[0], 128, 1);
  // A constant range widens instead of dividing by zero.
  EXPECT_EQ(cvc::color_slice(s, gray, 5.0, 5.0, true).size(), px.size());
}

// ---- T5: bad input ----

TEST(VolumeSlice, BadInputRaisesWithAUsefulMessage) {
  cvc::app app;
  const cvc::volume v = coded(app);
  try {
    cvc::extract_slice(v, cvc::slice_axis::z, 3);
    FAIL() << "slice 3 of 3 must raise";
  } catch (const std::out_of_range &e) {
    EXPECT_NE(std::string(e.what()).find("outside the 5x4x3 volume (0 to 2)"), std::string::npos)
        << e.what();
  }
  cvc::volume empty(app, cvc::dimension(0, 4, 4));
  EXPECT_THROW(cvc::extract_slice(empty, cvc::slice_axis::x, 0), std::invalid_argument);
  scratch_dir dir("bad");
  try {
    cvc::slice_source missing(app, dir.file("nope.rawiv"));
    FAIL() << "a missing file must raise";
  } catch (const std::runtime_error &e) {
    EXPECT_NE(std::string(e.what()).find("nope.rawiv"), std::string::npos) << e.what();
  }
  { std::ofstream(dir.file("garbage.rawiv"), std::ios::binary) << "not a volume"; }
  EXPECT_THROW(cvc::slice_source(app, dir.file("garbage.rawiv")), std::runtime_error);
  cvc::writeVolumeFile(app, v, dir.file("one.rawiv"));
  try {
    cvc::slice_source wrong_var(app, dir.file("one.rawiv"), 3, 0);
    FAIL() << "variable 3 of a one-variable file must raise";
  } catch (const std::runtime_error &e) {
    EXPECT_NE(std::string(e.what()).find("not variable 3"), std::string::npos) << e.what();
  }
  cvc::slice_source src(app, dir.file("one.rawiv"));
  EXPECT_THROW(src.slice(cvc::slice_axis::y, 4), std::out_of_range);
  const std::uint64_t outside[3] = {0, 0, 3};
  EXPECT_THROW(src.value(outside), std::out_of_range);
}

// ---- T9: region reads ----

TEST(VolumeSlice, RegionReadsGiveTheVolumesOwnPlanes) {
  cvc::app app;
  const cvc::volume v = coded(app, 6, 5, 4);
  scratch_dir dir("region");
  const std::string file = dir.file("coded.rawiv");
  cvc::writeVolumeFile(app, v, file);
  cvc::slice_source mem(v);
  cvc::slice_source disk(app, file, 0, 0, 3);
  EXPECT_TRUE(mem.in_memory());
  EXPECT_FALSE(disk.in_memory());
  EXPECT_EQ(disk.in_memory_volume(), nullptr);
  EXPECT_EQ(disk.grid().dim, mem.grid().dim);
  EXPECT_EQ(disk.grid().box, mem.grid().box);
  EXPECT_DOUBLE_EQ(disk.min(), 0.0);
  EXPECT_DOUBLE_EQ(disk.max(), code(5, 4, 3));
  std::size_t planes = 0;
  for (int a = 0; a < 3; ++a) {
    const auto axis = static_cast<cvc::slice_axis>(a);
    for (std::uint64_t s = 0; s < mem.grid().count(a); ++s, ++planes) {
      const auto want = mem.slice(axis, s);
      const auto got = disk.slice(axis, s);
      ASSERT_EQ(got->width, want->width);
      ASSERT_EQ(got->height, want->height);
      EXPECT_EQ(got->index, s);
      EXPECT_EQ(got->values, want->values) << "axis " << a << " slice " << s;
    }
  }
  EXPECT_EQ(disk.reads(), planes) << "one region read a plane";
  EXPECT_EQ(mem.reads(), 0u);
  // The cache: the last three planes asked for (z 3, 2, 1) come back without a read, the same
  // object each time.
  const auto again = disk.slice(cvc::slice_axis::z, 1);
  EXPECT_EQ(disk.reads(), planes);
  EXPECT_EQ(disk.slice(cvc::slice_axis::z, 1), again);
  disk.slice(cvc::slice_axis::x, 5); // long gone: read again
  EXPECT_EQ(disk.reads(), planes + 1);
  // A voxel's value, from the plane that holds it.
  const std::uint64_t ijk[3] = {3, 2, 1};
  EXPECT_EQ(disk.value(ijk, cvc::slice_axis::y), code(3, 2, 1));
  EXPECT_EQ(mem.value(ijk), code(3, 2, 1));
}

// ---- T6: no singletons ----

TEST(VolumeSliceNoSingletons, TwoAppsReadTheirOwnFiles) {
  scratch_dir dir("apps");
  cvc::app a, b;
  cvc::writeVolumeFile(a, coded(a, 4, 4, 4), dir.file("a.rawiv"));
  cvc::volume vb = coded(b, 3, 6, 2);
  cvc::writeVolumeFile(b, vb, dir.file("b.rawiv"));
  cvc::slice_source sa(a, dir.file("a.rawiv"));
  cvc::slice_source sb(b, dir.file("b.rawiv"));
  EXPECT_EQ(sa.slice(cvc::slice_axis::z, 3)->at(1, 2), code(1, 2, 3));
  EXPECT_EQ(sb.slice(cvc::slice_axis::y, 5)->at(2, 1), code(2, 5, 1));
  EXPECT_EQ(sb.grid().dim, vb.voxel_dimensions());
}

TEST(VolumeSliceNoSingletons, TwoThreadsCutWhatEachCutsAlone) {
  cvc::app app;
  const cvc::volume v = coded(app, 40, 30, 20);
  const cvc::tf::model gray = grayscale_opaque();
  auto run = [&](cvc::slice_axis a, std::uint64_t first, std::vector<std::uint8_t> &out) {
    cvc::slice_source src(v, 2);
    out.clear();
    for (std::uint64_t s = first; s < first + 10; ++s) {
      const std::vector<std::uint8_t> px =
          cvc::color_slice(*src.slice(a, s), gray, src.min(), src.max(), s % 2 == 0);
      out.insert(out.end(), px.begin(), px.end());
    }
  };
  std::vector<std::uint8_t> alone_a, alone_b, ta_out, tb_out;
  run(cvc::slice_axis::z, 3, alone_a);
  run(cvc::slice_axis::x, 20, alone_b);
  std::thread ta(run, cvc::slice_axis::z, 3, std::ref(ta_out));
  std::thread tb(run, cvc::slice_axis::x, 20, std::ref(tb_out));
  ta.join();
  tb.join();
  EXPECT_EQ(ta_out, alone_a);
  EXPECT_EQ(tb_out, alone_b);
  // A then B equals B then A.
  std::vector<std::uint8_t> b_first, a_second;
  run(cvc::slice_axis::x, 20, b_first);
  run(cvc::slice_axis::z, 3, a_second);
  EXPECT_EQ(a_second, alone_a);
  EXPECT_EQ(b_first, alone_b);
}

// ---- T8: the parameters on the state tree ----

TEST(VolumeSliceParams, RoundTripThroughTheTree) {
  cvc::app app;
  cvc::volume_slice_params p;
  p.axis = cvc::slice_axis::x;
  p.index = 7;
  p.filter = cvc::slice_filter::linear;
  p.opaque = false;
  p.overlay_offset = 4.5;
  cvc::state &node = app.root()("doc.scene.slice");
  cvc::params_to_state(p, node);
  EXPECT_EQ(node("axis").value(), "x");
  EXPECT_EQ(node("axis").property("choices"), "x,y,z");
  EXPECT_EQ(node("index").property("unit"), "voxel");
  EXPECT_EQ(node("filter").value(), "linear");
  const auto back = cvc::params_from_state<cvc::volume_slice_params>(node);
  ASSERT_TRUE(back.ok()) << back.message();
  EXPECT_EQ(back.value.axis, p.axis);
  EXPECT_EQ(back.value.index, 7);
  EXPECT_EQ(back.value.filter, p.filter);
  EXPECT_FALSE(back.value.opaque);
  EXPECT_DOUBLE_EQ(back.value.overlay_offset, 4.5);
  // Defaults seeded; a bad value reported, not replaced.
  cvc::state &fresh = app.root()("doc.scene.other");
  const auto seeded = cvc::params_from_state<cvc::volume_slice_params>(fresh);
  EXPECT_TRUE(seeded.ok()) << seeded.message();
  EXPECT_EQ(fresh("axis").value(), "z");
  EXPECT_EQ(fresh("index").value(), "-1");
  fresh("axis").value("w");
  fresh("index").value("-4");
  const auto bad = cvc::params_from_state<cvc::volume_slice_params>(fresh);
  EXPECT_FALSE(bad.ok());
  EXPECT_NE(bad.message().find("axis"), std::string::npos) << bad.message();
  EXPECT_NE(bad.message().find("index = -4 is below its minimum -1"), std::string::npos)
      << bad.message();
}
