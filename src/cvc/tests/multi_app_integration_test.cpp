/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Several cvc::apps in one process never collide. libcvc keeps no process-wide state: what an
// app registers, caches, counts or writes is its own (VOLROVER-LIBRARY-PORTS-ROADMAP.md 1.3,
// tools/singleton_scan/README.md).
//
// Each workload is what one app does, given its index, and returns a transcript of what that app
// saw. Every transcript must be the same
//   - when its app runs alone (each in a fresh app, one after another),
//   - when all the apps run at once, each on its own thread, released together,
//   - when the apps all exist at once and run one after another, in reverse order,
// and the transcripts of different indexes must differ, so an app that saw another's state would
// show it. Each subsystem that moves onto the app adds its part to the workload here.

#include <barrier>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/section.h>
#include <cvc/core/app.h>
#include <cvc/core/file_handler_registry.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/types.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_info.h>
#include <cvc/volume/volume_file_io.h>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <span>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace se = cvc::state_exec;

namespace {

constexpr int kApps = 6;

using workload = std::function<std::string(cvc::app &app, int index)>;

// ── the three ways of running a workload ──

std::vector<std::string> run_alone(const workload &w) {
  std::vector<std::string> out;
  for (int i = 0; i < kApps; ++i) {
    cvc::app app;
    out.push_back(w(app, i));
  }
  return out;
}

std::vector<std::string> run_together(const workload &w) {
  std::vector<std::string> out(kApps);
  std::barrier start(kApps);
  std::vector<std::thread> threads;
  for (int i = 0; i < kApps; ++i)
    threads.emplace_back([&, i] {
      cvc::app app;
      start.arrive_and_wait(); // every app made before any works
      out[i] = w(app, i);
    });
  for (std::thread &t : threads)
    t.join();
  return out;
}

std::vector<std::string> run_reversed(const workload &w) {
  std::vector<std::unique_ptr<cvc::app>> apps;
  for (int i = 0; i < kApps; ++i)
    apps.push_back(std::make_unique<cvc::app>());
  std::vector<std::string> out(kApps);
  for (int i = kApps - 1; i >= 0; --i)
    out[i] = w(*apps[i], i);
  return out;
}

void expect_no_collisions(const workload &w) {
  const std::vector<std::string> alone = run_alone(w);
  for (int i = 0; i < kApps; ++i)
    for (int j = i + 1; j < kApps; ++j)
      EXPECT_NE(alone[i], alone[j])
          << "apps " << i << " and " << j << " saw the same: the workload shows no app's own state";
  const std::vector<std::string> together = run_together(w);
  const std::vector<std::string> reversed = run_reversed(w);
  for (int i = 0; i < kApps; ++i) {
    EXPECT_EQ(together[i], alone[i]) << "app " << i << " on its thread, with the others at once";
    EXPECT_EQ(reversed[i], alone[i]) << "app " << i << " with the others alive, in reverse order";
  }
}

// ── the parts of the workload ──

// The file handler tables (#1119): a handler for ".app<i>" goes into app i only.
struct app_volume_io : cvc::volume_file_io {
  explicit app_volume_io(int index)
      : _id("app_volume_io_" + std::to_string(index)), _ext{".app" + std::to_string(index)} {}
  const std::string &id() const override { return _id; }
  const extension_list &extensions() const override { return _ext; }
  void getVolumeFileInfo(cvc::app &, cvc::volume_file_info::data &,
                         const std::string &) const override {}
  void readVolumeFile(cvc::app &, cvc::volume &, const std::string &, unsigned int, unsigned int,
                      cvc::uint64, cvc::uint64, cvc::uint64,
                      const cvc::dimension &) const override {}
  void createVolumeFile(cvc::app &, const std::string &, const cvc::bounding_box &,
                        const cvc::dimension &, const std::vector<cvc::data_type> &, unsigned int,
                        unsigned int, double, double) const override {}
  void writeVolumeFile(cvc::app &, const cvc::volume &, const std::string &, unsigned int,
                       unsigned int, cvc::uint64, cvc::uint64, cvc::uint64) const override {}
  std::string _id;
  extension_list _ext;
};

void file_handlers(cvc::app &app, int index, std::ostringstream &t) {
  app.volume_handlers().insert(cvc::volume_file_io::ptr(new app_volume_io(index)));
  t << "volume handlers for:";
  for (int j = 0; j < kApps; ++j)
    if (!app.volume_handlers().for_extension(".app" + std::to_string(j)).empty())
      t << " .app" << j;
  t << "\n";
}

// The action intrinsics (#1137): (who) answers this app's name, in this app's programs only.
void action_intrinsics(cvc::app &app, int index, std::ostringstream &t) {
  const std::string name = "app-" + std::to_string(index);
  cvc::ariadne::register_action_intrinsics(
      app, "multi-app.who", [name](std::shared_ptr<se::environment> env, se::intrinsics_context &) {
        se::builtins::register_fn(
            env, "who", [name](std::span<const se::value_t>) { return se::value_t(name); });
      });
  std::vector<std::string> errs;
  const bool ran = cvc::ariadne::run_init(app, "multi", "(state-set \"who\" (who))", &errs);
  t << "(who): " << (ran ? cvc::state::instance(app)("multi.who").value() : "error") << "\n";
}

// The state tree: a value written under the app's root is read back from it.
void state_tree(cvc::app &app, int index, std::ostringstream &t) {
  cvc::state::instance(app)("multi.value").value(std::to_string(index * 7));
  t << "multi.value: " << cvc::state::instance(app)("multi.value").value() << "\n";
}

// Sections (this PR): a writer's epoch is its app's, and each app's revisions count on their own.
void sections(cvc::app &app, int index, std::ostringstream &t) {
  cvc::ariadne::section_writer w(app, "multi.section");
  w.set({"v"}, std::to_string(index));
  const cvc::ariadne::section_stamp first = w.commit();
  const cvc::ariadne::section_stamp second = w.commit();
  t << "section: epoch is the app's " << (first.epoch == cvc::ariadne::section_epoch(app))
    << ", revisions " << first.rev << " " << second.rev << ", v "
    << cvc::state::instance(app)("multi.section.v").value() << "\n";
}

std::string whole_workload(cvc::app &app, int index) {
  std::ostringstream t;
  file_handlers(app, index, t);
  action_intrinsics(app, index, t);
  state_tree(app, index, t);
  sections(app, index, t);
  return t.str();
}

TEST(MultiAppIntegration, AppsNeverCollide) { expect_no_collisions(whole_workload); }

// A section epoch is the app's: two apps differ, one app keeps its own.
TEST(MultiAppIntegration, EachAppHasItsOwnSectionEpoch) {
  cvc::app a, b;
  EXPECT_NE(cvc::ariadne::section_epoch(a), cvc::ariadne::section_epoch(b));
  EXPECT_EQ(cvc::ariadne::section_epoch(a), cvc::ariadne::section_epoch(a));
  const std::uint64_t before = cvc::ariadne::section_epoch(a);
  {
    cvc::app between;
    (void)cvc::ariadne::section_epoch(between);
  }
  EXPECT_EQ(cvc::ariadne::section_epoch(a), before) << "making another app moved this one's";
}

} // namespace
