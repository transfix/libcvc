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

#include <algorithm>
#include <barrier>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/media_player.h>
#include <cvc/ariadne/section.h>
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_registry.h>
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/uri_state.h>
#include <cvc/core/app.h>
#include <cvc/core/file_handler_registry.h>
#include <cvc/geometry/geometry.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/sim_world.h>
#include <cvc/physics/world.h>
#include <cvc/physics/world_binding.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/types.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_info.h>
#include <cvc/volume/volume_file_io.h>
#include <cvc/world/generated_world.h>
#include <cvc/world/terrain_source.h>
#ifdef CVC_ENABLE_SDF
#include <cvc/utility/algorithm.h>
#endif
#include <cstdio>
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

// The document registries (Ariadne part 2): app i registers widget type w<i>, block b<i> and
// decoder d<i>; a document naming every app's shows which this app knows.
void document_registries(cvc::app &app, int index, std::ostringstream &t) {
  const std::string tag = std::to_string(index);
  cvc::ariadne::register_widget_type(
      app, "w" + tag, [](const cvc::ariadne::Widget &, const cvc::ariadne::WidgetEmitContext &) {});
  cvc::ariadne::register_ari_block(
      app, "b" + tag, [tag](const cvc::ariadne::Value &content, cvc::ariadne::LoadResult &out) {
        out.extras.emplace_back("b" + tag, content);
      });
  cvc::ariadne::register_data_decoder(app, "d" + tag, [](cvc::ariadne::DataDecode &) {});
  t << "registries:";
  for (int j = 0; j < kApps; ++j) {
    const std::string s = std::to_string(j);
    if (cvc::ariadne::has_widget_type(app, "w" + s))
      t << " w" << s;
    if (cvc::ariadne::has_ari_block(app, "b" + s))
      t << " b" << s;
    if (cvc::ariadne::has_data_decoder(app, "d" + s))
      t << " d" << s;
  }
  if (cvc::ariadne::have_yaml()) {
    std::string doc = "meta: { min_libcvc: \"0.0.0\" }\n";
    for (int j = 0; j < kApps; ++j)
      doc += "b" + std::to_string(j) + ": { from: " + std::to_string(j) + " }\n";
    const cvc::ariadne::LoadResult r = cvc::ariadne::load_string(app, doc);
    t << "; the document's blocks kept:";
    for (const auto &kv : r.extras)
      t << " " << kv.first;
  }
  t << "\n";
}

// Media (Ariadne part 2b): even apps play always, odd ones never, and only app 0 has a player.
void media(cvc::app &app, int index, std::ostringstream &t) {
  namespace am = cvc::ariadne::media;
  am::set_play_policy(app, index % 2 == 0 ? am::PlayPolicy::Always : am::PlayPolicy::Never);
  if (index == 0)
    am::register_player_factory(
        app, [](const am::PlayerSource &) { return std::unique_ptr<am::Player>(); });
  t << "media: may play " << am::may_play_here(app) << ", has a player "
    << am::have_player_factory(app) << "\n";
}

// Streams (Ariadne part 2b): a stream app i opens under a shared name is in its registry only.
void streams(cvc::app &app, int index, std::ostringstream &t) {
  namespace sx = cvc::ariadne::stream;
  sx::stream_params p;
  p.id = "shared";
  p.format.kind = sx::frame_kind::video_raw;
  p.format.codec = "rgba8";
  p.format.w = 2;
  p.format.h = 2;
  p.format.stride = 8;
  std::shared_ptr<sx::stream> s = sx::stream::open(app, p);
  t << "streams: opened " << static_cast<bool>(s);
  if (s) {
    sx::stream_registry &r = sx::stream_registry::for_app(app);
    t << ", in this app's registry " << (r.lookup(s->token()) != nullptr) << ", tokens " << r.size()
      << ", index " << index;
  }
  t << "\n";
}

// The URI resolver (Ariadne part 3): app i's "memo" scheme answers its index, its state:// reads
// its own tree (multi.value, which state_tree wrote), and its read cap is its own.
void uris(cvc::app &app, int index, std::ostringstream &t) {
  namespace ari = cvc::ariadne;
  const std::string mine = "memo of app " + std::to_string(index);
  ari::register_uri_handler(app, "memo", [mine](const ari::Uri &, const std::string &) {
    return ari::UriResult{true, mine, "memo://x", std::string()};
  });
  ari::register_state_uri_handler(app);
  ari::set_resolve_file_byte_cap(app, 1000 + static_cast<std::size_t>(index));
  t << "uris: " << ari::resolve(app, "memo://x").content << ", state "
    << ari::resolve(app, "state://multi.value").content << ", cap "
    << ari::resolve_file_byte_cap(app) << "\n";
}

// Nav, world, physics and geometry (no singletons, nav/world): app i registers a terrain
// generator of its own under a shared name and samples it, computes an SDF of its own size,
// steps a world of 6 + 2i agents with its tree's sensing range, and binds a physics world to its
// tree's gravity.
void nav_world(cvc::app &app, int index, std::ostringstream &t) {
  char b[160];
  // The terrain generators are the app's: "multi" is flat ground at 3i metres.
  const double z = 3.0 * index;
  cvc::world::register_terrain_generator(app, "multi", [z](const auto &q) {
    std::vector<float> h(9, float(z));
    return std::shared_ptr<const cvc::world::terrain_source>(
        std::make_shared<const cvc::world::raster_heightfield>(3, 3, -q.half_m, -q.half_m, q.half_m,
                                                               q.half_m, std::move(h), 0.0, 50.0));
  });
  cvc::world::generated_world_params gp;
  gp.terrain = "multi";
  t << "terrain generators:";
  for (const std::string &n : cvc::world::terrain_generator_names(app))
    t << " " << n;
  std::snprintf(b, sizeof(b), "; ground %.2f\n", cvc::world::make_terrain(app, gp)->sample(0, 0));
  t << b;
#ifdef CVC_ENABLE_SDF
  // An SDF of its own: a box of half-size 0.3 + 0.05i on a 16^3 or 32^3 grid.
  {
    const double h = 0.3 + 0.05 * index;
    cvc::geometry box;
    for (int k = 0; k < 8; ++k)
      box.points().push_back({{(k == 1 || k == 2 || k == 5 || k == 6) ? h : -h,
                               (k == 2 || k == 3 || k == 6 || k == 7) ? h : -h, k >= 4 ? h : -h}});
    const unsigned int tris[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
                                      {0, 1, 5}, {0, 5, 4}, {3, 7, 6}, {3, 6, 2},
                                      {0, 4, 7}, {0, 7, 3}, {1, 2, 6}, {1, 6, 5}};
    for (const auto &tr : tris)
      box.tris().push_back({{tr[0], tr[1], tr[2]}});
    const int n = index % 2 ? 32 : 16;
    const cvc::volume v = cvc::sdf(app, box, cvc::dimension(n, n, n),
                                   cvc::bounding_box(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0), cvc::SDF_V1);
    double lo = 1e30, sum = 0.0;
    for (cvc::uint64 i = 0; i < v.XDim() * v.YDim() * v.ZDim(); ++i) {
      lo = std::min(lo, v(i));
      sum += v(i);
    }
    std::snprintf(b, sizeof(b), "sdf: %llux%llux%llu, least %.4f, sum %.4f\n",
                  static_cast<unsigned long long>(v.XDim()),
                  static_cast<unsigned long long>(v.YDim()),
                  static_cast<unsigned long long>(v.ZDim()), lo, sum);
    t << b;
  }
#endif
  // A nav world: 6 + 2i agents, the sensing range its tree says, a few ticks, published back.
  {
    namespace nav = cvc::nav;
    const nav::training_scene ts = nav::city_scene(48);
    nav::sim_world::config cfg;
    cfg.rows = ts.rows;
    cfg.cols = ts.cols;
    cfg.min_x = ts.min_x;
    cfg.min_y = ts.min_y;
    cfg.max_x = ts.max_x;
    cfg.max_y = ts.max_y;
    cfg.cx = ts.cx;
    cfg.cy = ts.cy;
    cfg.scale = ts.scale;
    cfg.veh.rr = ts.rr;
    cfg.veh.d_hat = ts.d_hat;
    cfg.veh.dt = ts.dt;
    cfg.veh.vmax = ts.vmax;
    cvc::state::instance(app)("multi.nav.range_m").value(std::to_string(10 + 4 * index));
    nav::sim_world w = nav::sim_world::from_occupancy(
        cfg, ts.occ.data(), nav::coef_mlp::default_biased(), 6 + 2 * index, /*seed=*/3);
    w.pull(app, "multi");
    for (int k = 0; k < 6; ++k)
      w.step(1);
    w.publish(app, "multi");
    std::vector<float> pos(2 * w.size()), head(w.size());
    w.snapshot(pos.data(), head.data(), nullptr, nullptr, nullptr);
    double sum = 0.0;
    for (float p : pos)
      sum += p;
    const auto key = [&](const char *k) { return cvc::state::instance(app)(k).value(); };
    std::snprintf(b, sizeof(b), ", poses %.5f\n", sum);
    t << "nav: tick " << key("multi.nav.tick") << ", agents " << key("multi.nav.settings.agents")
      << ", range " << key("multi.nav.range_m") << b;
  }
  // A physics world bound to its tree: its gravity, stepped, published.
  {
    namespace ph = cvc::physics;
    cvc::state::instance(app)("multi.phys.backend").value(std::string("null"));
    cvc::state::instance(app)("multi.phys.gravity.z").value(std::to_string(-1.0 - index));
    ph::world w(ph::world_binding::config_from_state(app, "multi.phys"));
    ph::world_binding bind(app, w, "multi.phys");
    for (int k = 0; k < 3 + index; ++k)
      bind.step();
    t << "physics: gravity " << w.config().gravity.z << ", tick "
      << cvc::state::instance(app)("multi.phys.tick").value() << "\n";
  }
}

std::string whole_workload(cvc::app &app, int index) {
  std::ostringstream t;
  file_handlers(app, index, t);
  action_intrinsics(app, index, t);
  state_tree(app, index, t);
  sections(app, index, t);
  document_registries(app, index, t);
  media(app, index, t);
  streams(app, index, t);
  uris(app, index, t);
  nav_world(app, index, t);
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
