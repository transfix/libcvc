/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Several cvc::apps in one process never collide in cvcGL (tools/singleton_scan/README.md): the
// scene node types and shader presets an app registers, its mapper policy and draw path, the props
// it marks as casting no shadow, its publishers' flushes and its UI font are its own. Headless:
// nothing here opens a window or a GL context.
//
// The workload is what one app does, given its index, and returns a transcript of what that app
// saw (src/cvc/tests/multi_app_integration_test.cpp runs the core's the same way). Every
// transcript must be the same when its app runs alone, when all the apps run at once on threads,
// and when they all exist at once and run in reverse order; the transcripts of different apps must
// differ. Then a later app, made after the others are gone, starts clean.

#include <barrier>
#include <cstdint>
#include <cstdio>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/scene.h>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/LowMemoryPolyDataMapper.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/jpeg_codec.h>
#include <cvc/gl/state_publisher.h>
#include <cvc/state/state.h>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <vtkActor.h>
#include <vtkNew.h>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace {

namespace gla = cvc::gl::ariadne;
using cvc::gl::GeometryNode;
using cvc::gl::GraphicsNode;
using cvc::gl::LowMemoryMapperPolicy;
using cvc::gl::LowMemoryPolyDataMapper;
using cvc::gl::SceneGraph;

int failures = 0;

void chk(bool ok, const std::string &what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what.c_str());
  if (!ok)
    ++failures;
}

constexpr int kApps = 4;

using workload = std::function<std::string(cvc::app &app, int index)>;

// ── the three ways of running the workload ──

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

// ── the parts of the workload ──

cvc::geometry box(double s) {
  cvc::geometry g;
  const double v[8][3] = {{0, 0, 0}, {s, 0, 0}, {s, s, 0}, {0, s, 0},
                          {0, 0, s}, {s, 0, s}, {s, s, s}, {0, s, s}};
  for (const auto &p : v)
    g.points().push_back({p[0], p[1], p[2]});
  const int f[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
                        {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
  for (const auto &t : f)
    g.tris().push_back({static_cast<unsigned int>(t[0]), static_cast<unsigned int>(t[1]),
                        static_cast<unsigned int>(t[2])});
  return g;
}

std::string type_of(int index) { return "marker_" + std::to_string(index % kApps); }
std::string preset_of(int index) { return "tint_" + std::to_string(index % kApps); }

// The scene node types and shader presets: app i registers marker_i and tint_i, and a scene that
// also names app i+1's realizes only its own (the other's type is skipped with a warning, its
// preset unknown).
void scene_extensions(cvc::app &app, int index, std::ostringstream &t) {
  const std::string mine = type_of(index), theirs = type_of(index + 1);
  const std::string tint = preset_of(index), otherTint = preset_of(index + 1);
  gla::register_scene_node_type(
      app, mine,
      [index](SceneGraph &sg, const cvc::ariadne::SceneNode &n, GraphicsNode *parent,
              gla::RealizedScene &, std::vector<std::string> *) -> std::shared_ptr<GraphicsNode> {
        const cvc::geometry geom = box(1.0 + index);
        if (parent)
          return parent->createChild<GeometryNode>(n.id, geom);
        return sg.addGraphics(n.id, geom);
      });
  auto styled = std::make_shared<std::vector<std::string>>(); // the nodes tint_i ran on
  gla::register_shader_preset(app, tint,
                              [styled](GeometryNode &g) { styled->push_back(g.getName()); });
  t << "types: " << mine << " " << gla::has_scene_node_type(app, mine) << ", " << theirs << " "
    << gla::has_scene_node_type(app, theirs) << "; presets: " << tint << " "
    << gla::has_shader_preset(app, tint) << ", " << otherTint << " "
    << gla::has_shader_preset(app, otherTint) << "\n";

  SceneGraph sg(app, "multi");
  cvc::ariadne::Scene scene;
  cvc::ariadne::SceneNode own;
  own.id = "own";
  own.type = mine;
  own.shader.present = true;
  own.shader.preset = tint;
  cvc::ariadne::SceneNode foreign;
  foreign.id = "foreign";
  foreign.type = theirs;
  cvc::ariadne::SceneNode foreignTint;
  foreignTint.id = "foreign_tint";
  foreignTint.type = mine;
  foreignTint.shader.present = true;
  foreignTint.shader.preset = otherTint;
  scene.nodes = {own, foreign, foreignTint};
  std::vector<std::string> warnings;
  gla::RealizedScene realized = gla::realize_scene(sg, scene, "multi", &warnings);
  t << "realized: own " << sg.hasGraphics("own") << ", foreign " << sg.hasGraphics("foreign")
    << ", foreign_tint " << sg.hasGraphics("foreign_tint") << "; " << tint << " ran on";
  for (const std::string &s : *styled)
    t << " " << s;
  t << "\nwarnings:";
  for (const std::string &w : warnings)
    t << " [" << w << "]";
  t << "\n";

  cvc::ariadne::LoadResult needsMine, needsTheirs;
  needsMine.customs.push_back({cvc::ariadne::CustomRequirement::Kind::Node, mine, true});
  needsTheirs.customs.push_back({cvc::ariadne::CustomRequirement::Kind::Node, theirs, true});
  t << "customs: " << mine << " " << gla::verify_scene_customs(app, needsMine) << ", " << theirs
    << " " << gla::verify_scene_customs(app, needsTheirs) << "\n";
}

// The mapper policy and draw path: each app sets its own and reads its own back.
void mapper_settings(cvc::app &app, int index, std::ostringstream &t) {
  const LowMemoryMapperPolicy policy =
      index % 2 ? LowMemoryMapperPolicy::Off : LowMemoryMapperPolicy::Force;
  cvc::gl::setLowMemoryMapperPolicy(app, policy);
  LowMemoryPolyDataMapper::setDrawPath(app,
                                       static_cast<LowMemoryPolyDataMapper::DrawPath>(index % 3));
  LowMemoryPolyDataMapper::setPassProgramsEnabled(app, index % 2 == 0);
  vtkSmartPointer<vtkPolyDataMapper> m = cvc::gl::newPolyDataMapper(app);
  t << "mapper: policy " << static_cast<int>(cvc::gl::lowMemoryMapperPolicy(app)) << ", low-memory "
    << (LowMemoryPolyDataMapper::SafeDownCast(m) != nullptr) << ", draw path "
    << static_cast<int>(LowMemoryPolyDataMapper::drawPath(app)) << ", pass programs "
    << LowMemoryPolyDataMapper::passProgramsEnabled(app) << "\n";
}

// The props an app marks as casting no shadow.
void shadow_marks(cvc::app &app, int index, std::ostringstream &t) {
  vtkNew<vtkActor> marked, plain;
  if (index % 2 == 0)
    GraphicsNode::setPropCastsShadow(app, marked, false);
  t << "shadows: marked casts " << GraphicsNode::propCastsShadow(app, marked) << ", plain casts "
    << GraphicsNode::propCastsShadow(app, plain) << "\n";
}

// A publisher's flushes: the handler its writes fire sees its own publisher flushing, never
// another app's.
void publisher_flushes(cvc::app &app, int index, std::ostringstream &t) {
  cvc::gl::state_publisher pub(app);
  const std::string path = "multi.flush";
  cvc::state::instance(app)(path).value(std::string("0"));
  int own = 0, other = 0;
  boost::signals2::scoped_connection c =
      cvc::state::instance(app)(path).valueChanged.connect([&]() {
        if (cvc::gl::state_publisher::flushing(app) == &pub)
          ++own;
        else
          ++other;
      });
  for (int k = 1; k <= 3 + index; ++k) {
    pub.publish(path, std::to_string(k));
    pub.flush();
  }
  t << "flushes: own " << own << ", other " << other << ", last "
    << cvc::state::instance(app)(path).value() << "\n";
}

// The JPEG codec: each producer and view has its own (no handle shared between threads).
void jpeg(cvc::app &, int index, std::ostringstream &t) {
  cvc::gl::jpeg_encoder encoder;
  cvc::gl::jpeg_decoder decoder;
  const int w = 16 + 16 * index, h = 16;
  const std::vector<std::uint8_t> rgba(static_cast<std::size_t>(w) * h * 4, 128);
  std::vector<std::uint8_t> bytes(64 * 1024);
  const std::size_t n = encoder.encode(rgba.data(), w, h, 80, bytes.data(), bytes.size());
  std::vector<std::uint8_t> back;
  int bw = 0, bh = 0;
  const bool ok = n > 0 && decoder.decode(bytes.data(), n, back, bw, bh);
  t << "jpeg: " << (ok ? std::to_string(bw) + "x" + std::to_string(bh) : std::string("none"))
    << "\n";
}

std::string whole_workload(cvc::app &app, int index) {
  std::ostringstream t;
  scene_extensions(app, index, t);
  mapper_settings(app, index, t);
  shadow_marks(app, index, t);
  publisher_flushes(app, index, t);
  jpeg(app, index, t);
  return t.str();
}

void test_apps_never_collide() {
  std::printf("== the workload: alone, at once on threads, in reverse order ==\n");
  const std::vector<std::string> alone = run_alone(whole_workload);
  std::printf("%s", alone[0].c_str());
  for (int i = 0; i < kApps; ++i)
    for (int j = i + 1; j < kApps; ++j)
      chk(alone[i] != alone[j],
          "apps " + std::to_string(i) + " and " + std::to_string(j) + " saw different things");
  chk(alone[0].find("realized: own 1, foreign 0, foreign_tint 1; tint_0 ran on own\n") !=
          std::string::npos,
      "app 0 realized its own type with its own preset, and not app 1's type or preset");
  chk(alone[0].find("types: marker_0 1, marker_1 0; presets: tint_0 1, tint_1 0") !=
          std::string::npos,
      "app 0 has its own type and preset, not app 1's");
  chk(alone[0].find("customs: marker_0 1, marker_1 0") != std::string::npos,
      "app 0's customs gate passes its own type and fails app 1's");
  const std::vector<std::string> together = run_together(whole_workload);
  const std::vector<std::string> reversed = run_reversed(whole_workload);
  for (int i = 0; i < kApps; ++i) {
    chk(together[i] == alone[i], "app " + std::to_string(i) +
                                     " on its thread, with the others at once, saw what it does "
                                     "alone");
    chk(reversed[i] == alone[i],
        "app " + std::to_string(i) + " with the others alive, in reverse order, saw the same");
    if (together[i] != alone[i])
      std::printf("alone:\n%s\ntogether:\n%s\n", alone[i].c_str(), together[i].c_str());
    if (reversed[i] != alone[i])
      std::printf("alone:\n%s\nreversed:\n%s\n", alone[i].c_str(), reversed[i].c_str());
  }
}

// What one app set up is gone with it: an app made afterwards has none of its node types,
// presets, settings or marks.
void test_a_later_app_starts_clean() {
  std::printf("== a later app starts clean ==\n");
  vtkNew<vtkActor> prop;
  {
    cvc::app first;
    whole_workload(first, 0);
    GraphicsNode::setPropCastsShadow(first, prop, false);
    cvc::gl::setLowMemoryMapperPolicy(first, LowMemoryMapperPolicy::Off);
  }
  cvc::app later;
  chk(!gla::has_scene_node_type(later, type_of(0)) && !gla::has_shader_preset(later, preset_of(0)),
      "no node type or shader preset of the earlier app's");
  chk(GraphicsNode::propCastsShadow(later, prop), "a prop the earlier app marked casts here");
  cvc::app fresh;
  chk(cvc::gl::lowMemoryMapperPolicy(later) == cvc::gl::lowMemoryMapperPolicy(fresh),
      "the mapper policy is the default (the environment's), not the earlier app's Off");
}

// The UI font settings are the app's: an app that turned the fallback fonts off loads the UI font
// alone, while another app loads what an untouched app does.
void test_ui_fonts_per_app() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("SKIP the UI fonts: cvcGL was built without CVC_ENABLE_IMGUI\n");
#else
  std::printf("== the UI fonts are the app's ==\n");
  const auto sources = [](cvc::app &app) {
    ImGuiContext *ctx = ImGui::CreateContext();
    ImGui::SetCurrentContext(ctx);
    ImGuiIO &io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.BackendFlags |= ImGuiBackendFlags_RendererHasTextures;
    cvc::gl::ImGuiOverlay::loadUiFonts(app, io);
    const int n = io.Fonts->Sources.Size;
    ImGui::DestroyContext(ctx);
    return n;
  };
  cvc::app plain, other, untouched;
  cvc::gl::ImGuiOverlay::setUiFontFallbacks(plain, false);
  const int plainSources = sources(plain);
  const int otherSources = sources(other);
  chk(plainSources == 1, "fallbacks off: the UI font alone (" + std::to_string(plainSources) + ")");
  chk(otherSources == sources(untouched) && otherSources >= plainSources,
      "another app keeps the fallbacks on (" + std::to_string(otherSources) + " sources)");
#endif
}

} // namespace

int main() {
  test_apps_never_collide();
  test_a_later_app_starts_clean();
  test_ui_fonts_per_app();
  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures,
              failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
