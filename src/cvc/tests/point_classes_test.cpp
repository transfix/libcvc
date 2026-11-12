/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::point_classes (cvc/volume/point_classes.h, point_class_intrinsics.h): the VolumeRover
// ports' P0d rows for it (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.7).
//   - the model: add, remove, classes by name, variable and timestep, point_class_problems;
//   - T8: point_class's and point_class_read_params' tables on the state tree;
//   - T9: .vgr round-trips, the legacy samples (src/cvc/tests/data/point_classes) read, and
//     truncated and garbage files raise with the file's name;
//   - T6: two apps keep their own handlers; two threads with different parameters read what each
//     reads alone; A then B equals B then A;
//   - the DSL: point-classes.* and params.*, in an environment and from an Ariadne document
//     through the app's own provider (install_point_class_intrinsics).
// Runs natively and under node (the ports lane, T14).

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/core/param_dsl.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/volume/point_class_intrinsics.h>
#include <cvc/volume/point_classes.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace fs = std::filesystem;
namespace se = cvc::state_exec;
using steady = std::chrono::steady_clock;

fs::path data_dir() { return fs::path(__FILE__).parent_path() / "data" / "point_classes"; }

struct scratch_dir {
  fs::path path;
  explicit scratch_dir(const std::string &name) {
    path = fs::temp_directory_path() /
           ("cvc_points_" + name + "_" + std::to_string(steady::now().time_since_epoch().count()));
    fs::create_directories(path);
  }
  ~scratch_dir() {
    std::error_code ec;
    fs::remove_all(path, ec);
  }
  std::string file(const std::string &name) const { return (path / name).generic_string(); }
};

std::string slurp(const fs::path &p) {
  std::ifstream in(p, std::ios::binary);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

void spit(const fs::path &p, const std::string &text) {
  std::ofstream out(p, std::ios::binary | std::ios::trunc);
  out << text;
}

cvc::point_classes sample() {
  cvc::point_classes pc;
  pc.add("membrane", 12, 40, 7);
  pc.add("membrane", 13, 40, 7);
  pc.add("background", 0, 0, 0);
  pc.add("membrane", 1, 1, 1, 0, 2); // the same name for timestep 2: a class of its own
  pc.classes[1].radius = 2.5;
  pc.classes[1].color = {0, 128 / 255.0, 1};
  pc.add("a \"quoted\" <name> & more", 5, 6, 7, 1, 0);
  return pc;
}

// ---- the model ----

TEST(PointClasses, AddRemoveAndClassesByNameVariableAndTimestep) {
  cvc::point_classes pc = sample();
  EXPECT_EQ(pc.classes.size(), 4u);
  EXPECT_EQ(pc.point_count(), 5u);
  EXPECT_EQ(pc.class_index("membrane"), 0);
  EXPECT_EQ(pc.class_index("membrane", 0, 2), 2);
  EXPECT_EQ(pc.class_index("membrane", 1, 0), -1);
  EXPECT_EQ(pc.class_names(), (std::vector<std::string>{"membrane", "background", "membrane",
                                                        "a \"quoted\" <name> & more"}));
  EXPECT_EQ(pc.classes[0].points, (std::vector<int>{12, 40, 7, 13, 40, 7}));
  EXPECT_EQ(pc.classes[0].color, cvc::point_class_palette(0)) << "new classes take the palette";
  EXPECT_EQ(pc.classes[2].color, cvc::point_class_palette(2));
  EXPECT_NE(cvc::point_class_palette(0), cvc::point_class_palette(1));
  EXPECT_EQ(cvc::point_class_palette(8), cvc::point_class_palette(0)) << "it cycles";
  EXPECT_FALSE(pc.add("membrane", 12, 40, 7)) << "already there";
  EXPECT_TRUE(pc.classes[0].contains(13, 40, 7));
  EXPECT_TRUE(pc.remove("membrane", 12, 40, 7));
  EXPECT_FALSE(pc.remove("membrane", 12, 40, 7));
  EXPECT_FALSE(pc.remove("nothing", 0, 0, 0));
  EXPECT_EQ(pc.classes[0].points, (std::vector<int>{13, 40, 7}));
  cvc::point_class c;
  c.name = "background";
  c.radius = 4;
  pc.set_class(c);
  EXPECT_EQ(pc.classes.size(), 4u) << "replaced in its place";
  EXPECT_EQ(pc.classes[1].radius, 4.0);
  EXPECT_TRUE(pc.classes[1].points.empty());
  EXPECT_THROW(pc.add("", 0, 0, 0), std::invalid_argument);
  EXPECT_THROW(pc.add("a", -1, 0, 0), std::invalid_argument);
  EXPECT_THROW(pc.add("a", 0, 0, 0, -1, 0), std::invalid_argument);
  c.name.clear();
  EXPECT_THROW(pc.set_class(c), std::invalid_argument);
}

TEST(PointClasses, ProblemsNameTheClassAndPoint) {
  EXPECT_TRUE(cvc::point_class_problems(sample()).empty());
  EXPECT_TRUE(cvc::point_class_problems(sample(), {64, 64, 32}).empty());
  cvc::point_classes pc = sample();
  pc.classes[0].points.insert(pc.classes[0].points.end(), {12, 40, 7, -1, 0, 0});
  pc.classes[1].color = {1, 1};
  pc.classes[1].radius = 0;
  pc.classes.push_back(pc.classes[1]);
  pc.classes[2].points.push_back(9);
  const auto has = [](const std::vector<std::string> &p, const std::string &what) {
    for (const std::string &x : p)
      if (x.find(what) != std::string::npos)
        return true;
    return false;
  };
  std::vector<std::string> p = cvc::point_class_problems(pc);
  EXPECT_TRUE(has(p, "class 'membrane': point (12, 40, 7) is listed twice"));
  EXPECT_TRUE(has(p, "class 'membrane': point (-1, 0, 0) is not a voxel index"));
  EXPECT_TRUE(has(p, "class 'background': its colour is not three values in [0, 1]"));
  EXPECT_TRUE(has(p, "class 'background': radius 0 is not a positive number"));
  EXPECT_TRUE(has(p, "two classes are class 'background'"));
  EXPECT_TRUE(has(p, "class 'membrane' (variable 0, timestep 2): 4 indices, not i, j, k triples"));
  p = cvc::point_class_problems(sample(), {13, 64, 32});
  EXPECT_TRUE(has(p, "class 'membrane': point (13, 40, 7) is outside the 13x64x32 volume"));
}

// ---- T8: the tables on the state tree ----

TEST(PointClassParams, ClassMetadataRoundTripsThroughTheTree) {
  cvc::app app;
  cvc::point_class c;
  c.name = "seeds";
  c.color = {0.25, 0.5, 1.0 / 3.0};
  c.radius = 1.75;
  c.variable = 2;
  c.timestep = 9;
  c.points = {1, 2, 3}; // bulk data: not in the table
  cvc::state &node = app.root()("doc.grid.classes.0");
  cvc::params_to_state(c, node);
  EXPECT_EQ(node("radius").value(), "1.75");
  EXPECT_EQ(node("radius").property("unit"), "voxel");
  EXPECT_EQ(node("timestep").value(), "9");
  EXPECT_FALSE(node.findThroughLinks("points")) << "the points go on the data channel";
  const auto back = cvc::params_from_state<cvc::point_class>(node);
  ASSERT_TRUE(back.ok()) << back.message();
  EXPECT_EQ(back.value.name, c.name);
  EXPECT_EQ(back.value.color, c.color);
  EXPECT_EQ(back.value.radius, c.radius);
  EXPECT_EQ(back.value.variable, 2);
  EXPECT_EQ(back.value.timestep, 9);
  // Defaults seeded, bad values reported.
  cvc::state &fresh = app.root()("doc.grid.classes.1");
  const auto seeded = cvc::params_from_state<cvc::point_class>(fresh);
  EXPECT_FALSE(seeded.ok()) << "an empty name is a problem";
  EXPECT_EQ(fresh("color").value(), "[1, 0, 0]");
  EXPECT_EQ(fresh("radius").property("default"), "1");
  fresh("name").value("x");
  fresh("variable").value("-3");
  const auto bad = cvc::params_from_state<cvc::point_class>(fresh);
  EXPECT_NE(bad.message().find("variable = -3 is below its minimum 0"), std::string::npos)
      << bad.message();
}

TEST(PointClassParams, ReadParamsCheckTheGrid) {
  cvc::point_class_read_params p;
  EXPECT_TRUE(cvc::params_validate(p).empty());
  p.dims = {64, 64};
  p.bbox = {0, 0, 0, 1, 1, 0};
  std::string all;
  for (const std::string &x : cvc::params_validate(p))
    all += x + "\n";
  EXPECT_NE(all.find("dims: 2 values, not three"), std::string::npos) << all;
  EXPECT_NE(all.find("bbox: its max z is not above its min"), std::string::npos) << all;
  p.dims = {64, 64, 0};
  p.bbox.clear();
  all.clear();
  for (const std::string &x : cvc::params_validate(p))
    all += x + "\n";
  EXPECT_NE(all.find("dims[2] = 0 is below its minimum 1"), std::string::npos) << all;
  EXPECT_NE(all.find("dims and bbox go together"), std::string::npos) << all;
  cvc::app app;
  cvc::point_class_read_params q;
  q.dims = {11, 11, 11};
  q.bbox = {0, 0, 0, 10, 10, 10};
  q.timestep = 3;
  cvc::params_to_state(q, app.root()("doc.read"));
  const auto back = cvc::params_from_state<cvc::point_class_read_params>(app.root()("doc.read"));
  ASSERT_TRUE(back.ok()) << back.message();
  EXPECT_EQ(back.value.dims, q.dims);
  EXPECT_EQ(back.value.bbox, q.bbox);
  EXPECT_EQ(back.value.timestep, 3);
}

// ---- T9: the formats ----

TEST(PointClassFiles, TheAppReadsVgrAndPtsAndWritesVgr) {
  cvc::app app;
  EXPECT_EQ(cvc::point_class_read_extensions(app), (std::vector<std::string>{".pts", ".vgr"}));
  EXPECT_EQ(cvc::point_class_write_extensions(app), (std::vector<std::string>{".vgr"}));
}

TEST(PointClassFiles, VgrRoundTripsEverything) {
  cvc::app app;
  scratch_dir dir("vgr");
  const cvc::point_classes pc = sample();
  cvc::write_point_classes(app, pc, dir.file("s.vgr"));
  EXPECT_EQ(cvc::read_point_classes(app, dir.file("s.vgr")), pc)
      << "colours that are whole 255ths, radii, variables and timesteps";
  EXPECT_NE(slurp(dir.path / "s.vgr").find("<!DOCTYPE pointclassdoc>"), std::string::npos);
  // A colour between 255ths keeps its nearest.
  cvc::point_classes odd = pc;
  odd.classes[0].color = {0.3, 0.6, 0.9};
  cvc::write_point_classes(app, odd, dir.file("odd.vgr"));
  const cvc::point_classes back = cvc::read_point_classes(app, dir.file("odd.vgr"));
  for (int k = 0; k < 3; ++k)
    EXPECT_NEAR(back.classes[0].color[k], odd.classes[0].color[k], 0.5 / 255.0);
  // Classes with problems aren't written; a point list is read only.
  cvc::point_classes bad = pc;
  bad.classes[0].points.insert(bad.classes[0].points.end(), {12, 40, 7});
  EXPECT_THROW(cvc::write_point_classes(app, bad, dir.file("bad.vgr")), std::invalid_argument);
  EXPECT_THROW(cvc::write_point_classes(app, pc, dir.file("s.pts")), cvc::point_class_file_error);
}

TEST(PointClassFiles, TheLegacyVgrReads) {
  cvc::app app;
  const cvc::point_classes pc =
      cvc::read_point_classes(app, (data_dir() / "legacy.vgr").generic_string());
  ASSERT_EQ(pc.classes.size(), 3u);
  EXPECT_EQ(pc.classes[0].name, "Class 0");
  EXPECT_EQ(pc.classes[0].color, (std::vector<double>{1, 0, 0}));
  EXPECT_EQ(pc.classes[0].radius, 1.0) << "a legacy file has no radius";
  EXPECT_EQ(pc.classes[0].points, (std::vector<int>{10, 20, 5, 11, 20, 5}))
      << "the repeat reads once";
  EXPECT_EQ(pc.classes[1].name, "membrane & lumen");
  EXPECT_EQ(pc.classes[1].color, (std::vector<double>{0, 170 / 255.0, 1}));
  EXPECT_EQ(pc.class_index("Class 0", 0, 1), 2);
  EXPECT_EQ(pc.classes[2].points, (std::vector<int>{3, 3, 3}));
  // Given the volume, a point outside it raises.
  cvc::point_class_read_params tight;
  tight.dims = {11, 64, 64};
  tight.bbox = {0, 0, 0, 1, 1, 1};
  try {
    cvc::read_point_classes(app, (data_dir() / "legacy.vgr").generic_string(), tight);
    ADD_FAILURE() << "read";
  } catch (const cvc::point_class_file_error &e) {
    EXPECT_NE(std::string(e.what()).find("point (11, 20, 5) is outside the 11x64x64 volume"),
              std::string::npos)
        << e.what();
  }
}

TEST(PointClassFiles, TheLegacyPointListReads) {
  cvc::app app;
  const std::string pts = (data_dir() / "legacy.pts").generic_string();
  cvc::point_class_read_params grid;
  grid.dims = {11, 11, 11};
  grid.bbox = {0, 0, 0, 10, 10, 10};
  grid.timestep = 4;
  const cvc::point_classes pc = cvc::read_point_classes(app, pts, grid);
  ASSERT_EQ(pc.classes.size(), 2u);
  EXPECT_EQ(pc.class_names(), (std::vector<std::string>{"Class 0", "Class 1"}));
  EXPECT_EQ(pc.classes[0].points, (std::vector<int>{0, 0, 0, 10, 10, 10, 5, 2, 7}))
      << "truncated, as VolumeGridRover computed them";
  EXPECT_EQ(pc.classes[1].points, (std::vector<int>{9, 0, 5}));
  EXPECT_EQ(pc.classes[1].timestep, 4);
  EXPECT_EQ(pc.classes[1].color, cvc::point_class_palette(1));
  // Without a grid: the nearest index.
  const cvc::point_classes nearest = cvc::read_point_classes(app, pts);
  EXPECT_EQ(nearest.classes[0].points, (std::vector<int>{0, 0, 0, 10, 10, 10, 5, 3, 8}));
  EXPECT_EQ(nearest.classes[1].points, (std::vector<int>{10, 0, 5}));
}

TEST(PointClassFiles, TruncatedAndGarbageFilesRaise) {
  cvc::app app;
  scratch_dir dir("bad");
  cvc::write_point_classes(app, sample(), dir.file("good.vgr"));
  const std::string vgr = slurp(dir.path / "good.vgr");
  struct bad_file {
    std::string name, text, says;
  };
  const std::vector<bad_file> cases = {
      {"half.vgr", vgr.substr(0, vgr.size() / 2), "not XML"},
      {"garbage.vgr", "\x01\x02 not xml", "not XML"},
      {"other.vgr", "<something/>", "not a point class file"},
      {"noname.vgr", "<pointclassdoc><pointclass color=\"#ff0000\"/></pointclassdoc>",
       "a <pointclass> has no name"},
      {"color.vgr", "<pointclassdoc><pointclass name=\"a\" color=\"red\"/></pointclassdoc>",
       "color 'red' is not #rrggbb"},
      {"point.vgr",
       "<pointclassdoc><pointclass name=\"a\" color=\"#ff0000\"><point>1 2</point>"
       "</pointclass></pointclassdoc>",
       "point '1 2' is not three voxel indices"},
      {"negative.vgr",
       "<pointclassdoc><pointclass name=\"a\" color=\"#ff0000\"><point>1 -2 3</point>"
       "</pointclass></pointclassdoc>",
       "is not three voxel indices"},
      {"radius.vgr",
       "<pointclassdoc><pointclass name=\"a\" color=\"#ff0000\" radius=\"0\"/></pointclassdoc>",
       "radius '0' is not a positive number"},
      {"short.pts", "3\n0 0 0\n1 1 1\n", "the file ends before"},
      {"letters.pts", "1\n0 zero 0\n", "'zero' is not a number"},
      {"count.pts", "-2\n", "'-2' is not a point count"},
      {"negative.pts", "1\n-3 0 0\n", "is not a voxel index"},
  };
  for (const bad_file &b : cases) {
    spit(dir.path / b.name, b.text);
    try {
      cvc::read_point_classes(app, dir.file(b.name));
      ADD_FAILURE() << b.name << " read";
    } catch (const cvc::point_class_file_error &e) {
      const std::string what = e.what();
      EXPECT_NE(what.find(b.says), std::string::npos) << b.name << ": " << what;
      EXPECT_NE(what.find(b.name), std::string::npos) << b.name << ": names the file: " << what;
    }
  }
  EXPECT_THROW(cvc::read_point_classes(app, dir.file("nothing.vgr")), cvc::point_class_file_error);
  EXPECT_THROW(cvc::read_point_classes(app, dir.file("x.xyz")), cvc::point_class_file_error);
  cvc::point_class_read_params p;
  p.dims = {1, 2};
  EXPECT_THROW(cvc::read_point_classes(app, dir.file("good.vgr"), p), std::invalid_argument);
}

// ---- T6: no singletons ----

struct fake_io : cvc::point_class_file_io {
  const std::string &id() const override { return _id; }
  const std::vector<std::string> &extensions() const override { return _ext; }
  bool can_read() const override { return true; }
  bool can_write() const override { return false; }
  cvc::point_classes read(cvc::app &, const std::string &,
                          const cvc::point_class_read_params &) const override {
    cvc::point_classes pc;
    pc.add("fake", 1, 2, 3);
    return pc;
  }
  std::string _id = "fake : v1";
  std::vector<std::string> _ext{".fake"};
};

TEST(PointClassNoSingletons, TwoAppsKeepTheirOwnHandlers) {
  cvc::app a, b;
  ASSERT_TRUE(cvc::point_class_handlers(a).insert(
      boost::shared_ptr<cvc::point_class_file_io>(new fake_io)));
  EXPECT_EQ(cvc::read_point_classes(a, "x.fake").class_names(), (std::vector<std::string>{"fake"}));
  EXPECT_THROW(cvc::read_point_classes(b, "x.fake"), cvc::point_class_file_error);
  EXPECT_EQ(cvc::point_class_handlers(a).size(), cvc::point_class_handlers(b).size() + 1);
}

TEST(PointClassNoSingletons, TwoThreadsReadWhatEachReadsAlone) {
  const std::string pts = (data_dir() / "legacy.pts").generic_string();
  cvc::app app;
  cvc::point_class_read_params pa, pb;
  pb.dims = {11, 11, 11};
  pb.bbox = {0, 0, 0, 10, 10, 10};
  pb.variable = 1;
  const cvc::point_classes alone_a = cvc::read_point_classes(app, pts, pa);
  const cvc::point_classes alone_b = cvc::read_point_classes(app, pts, pb);
  ASSERT_NE(alone_a, alone_b);
  EXPECT_EQ(cvc::read_point_classes(app, pts, pb), alone_b);
  EXPECT_EQ(cvc::read_point_classes(app, pts, pa), alone_a);
  scratch_dir dir("threads");
  std::atomic<int> mismatches{0};
  const auto run = [&](const cvc::point_class_read_params &p, const cvc::point_classes &want,
                       const std::string &out) {
    for (int i = 0; i < 50; ++i) {
      const cvc::point_classes got = cvc::read_point_classes(app, pts, p);
      if (got != want)
        ++mismatches;
      cvc::write_point_classes(app, got, out);
      if (cvc::read_point_classes(app, out) != want)
        ++mismatches;
    }
  };
  std::thread ta(run, std::cref(pa), std::cref(alone_a), dir.file("a.vgr"));
  std::thread tb(run, std::cref(pb), std::cref(alone_b), dir.file("b.vgr"));
  ta.join();
  tb.join();
  EXPECT_EQ(mismatches.load(), 0);
}

// ---- the DSL ----

class PointClassDsl : public ::testing::Test {
protected:
  void SetUp() override {
    ictx.sched = &app.exec_scheduler();
    ictx.root = &cvc::state::instance(app);
    ictx.tracker = &tracker;
    ictx.host = &app.exec_async_host();
    ictx.proc = se::make_process();
    ictx.proc->status = se::process_status::ready;
    env = se::builtins::make_default_environment();
    se::register_intrinsics(env, &ictx);
    cvc::register_point_class_intrinsics(env, ictx, app);
  }
  std::string run(const std::string &program) {
    se::execute_options eo;
    eo.env = env;
    se::async_scheduler &sched = app.exec_scheduler();
    const int pid = sched.execute(program, eo);
    const auto t0 = steady::now();
    while (std::chrono::duration<double>(steady::now() - t0).count() < 30.0) {
      const auto info = sched.get_process_info(pid);
      if (!info || se::is_finished(info->status))
        return info ? info->exit_error.value_or(std::string()) : std::string();
      sched.drain_ingress();
      sched.wake_awaiting();
      sched.sync_step();
      sched.sync_run(200000, 0.1);
    }
    return "timed out";
  }
  std::string key(const std::string &k) { return app.root()(k).value(); }
  const cvc::point_classes *stored(const std::string &path) {
    const boost::any d = app.root()(path).data();
    const se::value_t *v = boost::any_cast<se::value_t>(&d);
    if (!v)
      return nullptr;
    const se::data_object_ptr *o = std::get_if<se::data_object_ptr>(&v->v);
    if (!o || !*o)
      return nullptr;
    const auto *p = boost::any_cast<std::shared_ptr<const cvc::point_classes>>(&(*o)->payload);
    return p ? p->get() : nullptr;
  }

  cvc::app app;
  se::memory_tracker tracker;
  se::intrinsics_context ictx;
  se::environment_ptr env;
};

TEST_F(PointClassDsl, EditReadWriteAndInfo) {
  scratch_dir dir("dsl");
  const std::string out = dir.file("seeds.vgr");
  ASSERT_EQ(
      run("(begin"
          "  (state-data-set \"g.points\" (point-classes.add nil \"seeds\" 1 2 3))"
          "  (state-data-set \"g.points\" (point-classes.add \"g.points\" \"seeds\" 4 5 6))"
          "  (state-data-set \"g.points\" (point-classes.add \"g.points\" \"bg\" 0 0 0 0 2))"
          "  (state-data-set \"g.points\" (point-classes.remove \"g.points\" \"seeds\" 1 2 3))"
          "  (state-set \"r.points\" (point-classes.points \"g.points\" \"seeds\"))"
          "  (state-set \"r.bg\" (point-classes.points \"g.points\" \"bg\" 0 2))"
          "  (let ((i (point-classes.info \"g.points\")))"
          "    (state-set \"r.total\" (get-attr i \"points\"))"
          "    (state-set \"r.second\" (get-attr (nth (get-attr i \"classes\") 1) "
          "\"timestep\")))"
          "  (state-set \"r.written\" (point-classes.write \"g.points\" \"" +
          out +
          "\"))"
          "  (state-set \"r.back\" (get-attr (point-classes.info (point-classes.read \"" +
          out + "\")) \"points\")))"),
      "");
  const cvc::point_classes *pc = stored("g.points");
  ASSERT_NE(pc, nullptr);
  EXPECT_EQ(pc->classes.size(), 2u);
  EXPECT_EQ(pc->classes[0].points, (std::vector<int>{4, 5, 6}));
  EXPECT_EQ(key("r.total"), "2");
  EXPECT_EQ(key("r.second"), "2");
  EXPECT_EQ(key("r.written"), out);
  EXPECT_EQ(key("r.back"), "2");
  EXPECT_EQ(cvc::read_point_classes(app, out), *pc);
  // The .pts reader through a parameter dict.
  const std::string pts = (data_dir() / "legacy.pts").generic_string();
  ASSERT_EQ(run("(state-set \"r.pts\" (point-classes.points (point-classes.read \"" + pts +
                "\" (dict \"dims\" (list 11 11 11) \"bbox\" (list 0 0 0 10 10 10))) \"Class 1\"))"),
            "");
  EXPECT_NE(key("r.pts").find("9"), std::string::npos) << key("r.pts");
}

TEST_F(PointClassDsl, TheOriginalIsNeverChanged) {
  cvc::point_classes pc;
  pc.add("a", 1, 1, 1);
  app.root()("g.cpp").data(boost::any(pc));
  ASSERT_EQ(run("(state-data-set \"g.more\" (point-classes.add \"g.cpp\" \"a\" 2 2 2))"), "");
  const boost::any d = app.root()("g.cpp").data();
  EXPECT_EQ(boost::any_cast<cvc::point_classes>(d).point_count(), 1u);
  ASSERT_NE(stored("g.more"), nullptr);
  EXPECT_EQ(stored("g.more")->point_count(), 2u);
}

TEST_F(PointClassDsl, ParamsAndErrors) {
  ASSERT_EQ(run("(begin"
                "  (state-set \"r.radius\" (get-attr (params.defaults \"point-classes.class\") "
                "\"radius\"))"
                "  (state-set \"r.check\" (params.check \"point-classes.read\" "
                "(dict \"dims\" (list 1 2))))"
                "  (state-set \"r.missing\" (try (point-classes.read \"/no/such/file.vgr\")"
                "                             (catch (RuntimeError e) (get-attr e \"message\"))))"
                "  (state-set \"r.neg\" (try (point-classes.add nil \"a\" -1 0 0)"
                "                         (catch (ValueError e) (get-attr e \"message\"))))"
                "  (state-set \"r.float\" (try (point-classes.add nil \"a\" 1.5 0 0)"
                "                           (catch (TypeError e) (get-attr e \"message\"))))"
                "  (state-set \"r.arity\" (try (point-classes.add nil \"a\" 1 2)"
                "                           (catch (TypeError e) (get-attr e \"message\"))))"
                "  (state-set \"r.noclass\" (try (point-classes.points "
                "(point-classes.add nil \"a\" 1 2 3) \"b\")"
                "                             (catch (ValueError e) (get-attr e \"message\")))))"),
            "");
  EXPECT_EQ(key("r.radius"), "1");
  EXPECT_NE(key("r.check").find("dims: 2 values, not three"), std::string::npos) << key("r.check");
  EXPECT_NE(key("r.missing").find("/no/such/file.vgr"), std::string::npos) << key("r.missing");
  EXPECT_NE(key("r.neg").find("is not a voxel index"), std::string::npos) << key("r.neg");
  EXPECT_NE(key("r.float").find("i (an integer)"), std::string::npos) << key("r.float");
  EXPECT_NE(key("r.arity").find("takes point classes (or nil)"), std::string::npos)
      << key("r.arity");
  EXPECT_NE(key("r.noclass").find("no class 'b'"), std::string::npos) << key("r.noclass");
}

// ---- Ariadne: the app's own provider ----

struct ClickBackend : cvc::ariadne::Backend {
  std::string click;
  cvc::ariadne::Capabilities capabilities() const override {
    cvc::ariadne::Capabilities c;
    c.windows = true;
    return c;
  }
  bool begin_main_menu_bar() override { return false; }
  void end_main_menu_bar() override {}
  bool begin_menu(const char *) override { return false; }
  void end_menu() override {}
  bool begin_window(const char *, const char *, const cvc::ariadne::Size &, float,
                    bool *) override {
    return true;
  }
  void end_window() override {}
  void push_id(const char *) override {}
  void pop_id() override {}
  bool begin_grid(const cvc::ariadne::Layout &, const char *) override { return true; }
  void grid_next_cell() override {}
  void end_grid() override {}
  void text_line(const char *) override {}
  void text_value(const char *, const std::string &) override {}
  void separator() override {}
  bool button(const char *l) override {
    if (click == l) {
      click.clear();
      return true;
    }
    return false;
  }
  bool menu_item_action(const char *) override { return false; }
  cvc::ariadne::BoolEdit menu_item_toggle(const char *, bool) override { return {}; }
  cvc::ariadne::BoolEdit checkbox(const char *, bool) override { return {}; }
  cvc::ariadne::IntEdit slider_int(const char *, const char *, int, int, int) override {
    return {};
  }
  cvc::ariadne::DoubleEdit slider_double(const char *, const char *, double, double, double,
                                         const char *) override {
    return {};
  }
  cvc::ariadne::IndexEdit combo(const char *, int, const std::vector<std::string> &) override {
    return {};
  }
  cvc::ariadne::StringEdit input_text(const char *, const char *, const std::string &,
                                      bool) override {
    return {};
  }
};

void click_once(cvc::app &app, const std::string &action) {
  cvc::ariadne::Runtime rt(app, "");
  ClickBackend be;
  rt.set_backend(&be);
  rt.set_root(cvc::ariadne::group({cvc::ariadne::button("Mark", action)}));
  be.click = "Mark";
  for (int i = 0; i < 20; ++i) {
    rt.render();
    rt.drain();
  }
}

TEST(PointClassAriadne, AnActionMarksAPointThroughTheAppsOwnProvider) {
  const std::string action =
      "(begin (state-data-set \"grid.points\" (point-classes.add nil \"seeds\" 3 4 5))"
      "       (state-set \"grid.count\" (get-attr (point-classes.info \"grid.points\") "
      "\"points\")))";
  cvc::app with, without;
  cvc::install_point_class_intrinsics(with);
  click_once(with, action);
  EXPECT_EQ(with.root()("grid.count").value(), "1");
  click_once(without, action);
  EXPECT_TRUE(without.root()("grid.count").value().empty()) << "another app's documents lack them";
  EXPECT_EQ(cvc::find_param_table(without, "point-classes.class"), nullptr);
  cvc::ariadne::clear_action_intrinsics(with);
}

} // namespace
