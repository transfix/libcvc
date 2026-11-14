/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcgl_ariadne_ribbons -- `type: ribbon` and `type: draped_link` (ribbon_nodes.h, ARIADNE.md
// §5.4), loaded from a document and rendered for real:
//
//   append   a track whose state key grows by a point a frame uploads that point's quad alone
//            (48 B, one GL call, nothing created) -- never the whole node -- counted by the node
//            and at the driver; from text and from data() floats alike;
//   tail     a list whose LAST point changes rewrites only from it;
//   window   window: { from: <key>, units: m } moves what is drawn with no upload;
//   pixels   the track draws in its colour where its points are, and nothing past its window;
//   value    fields: xyv + ramp: colours each point by its value, the values uploaded per point;
//   width    a w field gives each point its own width;
//   stream   a stream of float32 frames appends each frame's points;
//   link     a draped_link between two nodes hugs the ground and follows the nodes with no upload;
//            value: + ramp: colours it, also with no upload;
//   regen    a ground made again (new heights) re-drapes the ribbons and links on it;
//   loader   a custom type's source: reaches its props; visible: and tags: apply.
//
// Skips (rc 0) where nothing rasterises unless CVC_REQUIRE_RENDER=1. Synthetic data only.

#include "streaming_test_util.h"

#include <boost/any.hpp>
#include <cmath>
#include <cstring>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/geometry/height_field.h>
#include <cvc/gl/DrapedLinkNode.h>
#include <cvc/gl/HeightFieldTexture.h>
#include <cvc/gl/RibbonNode.h>
#include <cvc/gl/ariadne/ribbon_nodes.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace cvcgl_test;
using cvc::gl::DrapedLinkNode;
using cvc::gl::RibbonNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
namespace ari = cvc::gl::ariadne;
namespace sx = cvc::ariadne::stream;

namespace {

const std::string kPrefix = "doc";
constexpr int W = 240, H = 240;

std::string key(const std::string &k) { return kPrefix + "." + k; }

void put_text(cvc::app &app, const std::string &k, const std::string &v) {
  cvc::ariadne::write_node(app, key(k)).value(v);
}
void put_floats(cvc::app &app, const std::string &k, const std::vector<float> &v) {
  cvc::ariadne::write_node(app, key(k))
      .data(boost::any(std::make_shared<const std::vector<float>>(v)));
}

std::string csv(const std::vector<float> &v) {
  std::ostringstream os;
  for (std::size_t i = 0; i < v.size(); ++i)
    os << (i ? "," : "") << v[i];
  return os.str();
}

// The track: east along y = -60 with a wiggle, a point every 4 m, at z = 1.
std::vector<float> track_points(int n) {
  std::vector<float> v;
  for (int k = 0; k < n; ++k) {
    v.push_back(static_cast<float>(-90 + 4 * k));
    v.push_back(static_cast<float>(-60 + 6 * std::sin(0.3 * k)));
    v.push_back(1.0f);
  }
  return v;
}

std::shared_ptr<RibbonNode> ribbon(SceneGraph &sg, const std::string &id) {
  return std::dynamic_pointer_cast<RibbonNode>(sg.getGraphics(id));
}
std::shared_ptr<DrapedLinkNode> link(SceneGraph &sg, const std::string &id) {
  return std::dynamic_pointer_cast<DrapedLinkNode>(sg.getGraphics(id));
}

// The scene: a dome on a 200 m ground (drawn far below, out of the way, so the overlays read
// against the black background), the ribbons and the links over it.
const char *kDoc = R"(
meta: { min_libcvc: "0.0.0" }
data:
  route_csv: { uri: route.csv, as: text }
scene:
  nodes:
    - node: ground
      source:
        heightfield:
          size: 200
          resolution: 41
          layers: [{ kind: dome, amplitude: 20, radius: 40 }]
      visible: false
    - node: track
      type: ribbon
      source: { state: tracks.a }
      width: 4
      color: [1, 0, 0]
      capacity: 64
      tags: [sim]
    - node: floats
      type: ribbon
      source: { state: tracks.b }
      width: 2
      color: [1, 0, 0]
      visible: show.floats
    - node: route
      type: ribbon
      points: [[-90, 60, 1], [0, 60, 1], [90, 60, 1]]
      width: 6
      color: "#00ff00"
      window: { from: route.from, units: m }
    - node: valued
      type: ribbon
      source: { state: tracks.v }
      fields: xyv
      width: 8
      ramp: [[0, [1, 0, 0]], [1, [0, 0, 1]]]
      lift: 1
    - node: widths
      type: ribbon
      source: { state: tracks.w }
      fields: xyzw
      color: [1, 1, 1]
    - node: draped
      type: ribbon
      source: { state: tracks.d }
      fields: xy
      ground: ground
      lift: 0.5
      color: c3
    - node: dataset
      type: ribbon
      source: { data: route_csv, item: text }
      width: 4
      color: [0, 1, 1]
    - node: streamed
      type: ribbon
      source: { stream: pts }
      color: [1, 1, 0]
    - node: car_a
      source: { box: 1 }
      transform: { position: [-40, 0, 0] }
      visible: false
    - node: car_b
      source: { box: 1 }
      transform: { position: [40, 0, 0] }
      visible: false
    - node: link
      type: draped_link
      ground: ground
      between: [car_a, car_b]
      stations: 25
      width: 4
      lift: 1
      value: links.q
      ramp: [[0, [1, 0, 0]], [1, [0, 1, 0]]]
)";

// A sensor stream of float32 points, at most 16 a frame.
std::unique_ptr<sx::stream> open_points(cvc::app &app) {
  sx::stream_params p;
  p.id = "pts";
  p.format.kind = sx::frame_kind::sensor;
  p.format.codec = "f32";
  p.format.bytes = 16 * 3 * sizeof(float);
  p.subscriber_depth = 4;
  p.expected_subscribers = 1;
  return sx::stream::open(app, p);
}

void publish_points(sx::stream &s, const std::vector<float> &xyz) {
  auto l = s.channel().pool().acquire();
  if (!check(l.has_value(), "stream: a frame slab is free"))
    return;
  std::memcpy(l->data, xyz.data(), xyz.size() * sizeof(float));
  s.channel().publish(*l, xyz.size() * sizeof(float), 0.0);
}

// RibbonNode's value colour and per-centre widths on one mapper family (the low-memory one is
// what WebGL2 draws with): a ramp's two ends in their colours, a replaceFrom that uploads its
// centres alone, positions and value texels both.
void testValuesDirect(cvc::app &app, cvc::gl::StreamingMapperKind kind) {
  std::printf("values: RibbonNode::setValueRamp [%s]\n", kindName(kind));
  SceneGraph sg(app, std::string("values_") + kindName(kind));
  sg.setDiagnosticChromeVisible(false);
  auto rib = sg.getGraphicsRoot()->addGraphicsChild<RibbonNode>(
      "valued", 64, 3.0f, cvc::bounding_box(-100, -100, -1, 100, 100, 1), kind);
  rib->setValueRamp({{0.0f, 1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f, 1.0f}});
  std::vector<float> xyz, halfs, values;
  for (int k = 0; k <= 20; ++k) {
    xyz.insert(xyz.end(), {static_cast<float>(-80 + 8 * k), 0.0f, 0.0f});
    halfs.push_back(k < 10 ? 2.0f : 8.0f);
    values.push_back(k < 10 ? 0.0f : 1.0f);
  }
  rib->replaceFrom(0, xyz.data(), 21, halfs.data(), values.data());
  SceneRenderer sr(sg, W, H, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  topView(sr, 0, 0, 100);
  Frame f = grab(sr);
  check(hueNear(f, toPx(sr, -50, 0, 0), 1, 0) && hueNear(f, toPx(sr, 50, 0, 0), 1, 2),
        "values: the ends draw the ramp's red and blue");
  check(!isBg(f.at(int(toPx(sr, 50, 6, 0)[0]), int(toPx(sr, 50, 6, 0)[1]))) &&
            isBg(f.at(int(toPx(sr, -50, 6, 0)[0]), int(toPx(sr, -50, 6, 0)[1]))),
        "values: the wide half is wide, the narrow half narrow");
  glcountInstall();
  const cvc::gl::StreamStats s0 = rib->streamStats();
  const RibbonNode::ValueStats v0 = rib->valueStats();
  const GLCount g0 = glcountRead();
  const float tail[3] = {90.0f, 0.0f, 0.0f}, w = 8.0f, v = 0.0f;
  rib->replaceFrom(21, tail, 1, &w, &v);
  f = grab(sr);
  const GLCount d = glcountRead() - g0;
  const cvc::gl::StreamStats s1 = rib->streamStats();
  const RibbonNode::ValueStats v1 = rib->valueStats();
  check(s1.uploads - s0.uploads == 1 && s1.uploadBytes - s0.uploadBytes == 48 &&
            s1.fullUploads == s0.fullUploads,
        "values: an appended centre uploads its quad");
  check(v1.fullUploads == v0.fullUploads && v1.uploads - v0.uploads == 1 &&
            v1.bytes - v0.bytes == sizeof(float),
        "values: and its one value texel");
  check(d.createdNothing(), "values: nothing created", glcountStr(d));
  check(hueNear(f, toPx(sr, 88, 0, 0), 1, 0), "values: the new end blends toward its red");
}

} // namespace

int main() {
  installErrorCounter();
  cvc::app app;
  std::printf("cvcgl_ariadne_ribbons\n");
  if (!renderAvailable(app))
    return finish("cvcgl_ariadne_ribbons");
  ari::register_ribbon_node_types(app);

  // --- loader: a custom type's source: is in its props, and the built-ins still parse it ----
  cvc::ariadne::LoadResult doc = cvc::ariadne::load_string(app, kDoc);
  check(doc.ok, "the document loads", doc.error);
  const cvc::ariadne::SceneNode *trackSpec = nullptr;
  for (const auto &n : doc.scene.nodes)
    if (n.id == "track")
      trackSpec = &n;
  check(trackSpec && trackSpec->props.find("source") &&
            trackSpec->props.find("source")->str("state") == "tracks.a",
        "loader: a ribbon's source: { state } is in its props");

  std::unique_ptr<sx::stream> pts = open_points(app);
  check(static_cast<bool>(pts), "stream: the points stream opens");

  // The dataset: a text file of numbers beside the document.
  std::random_device rd;
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() /
      ("cvcgl-ribbons-" + std::to_string(rd()) + std::to_string(rd()));
  std::filesystem::create_directories(dir);
  std::ofstream(dir / "route.csv") << "-90, -30, 1\n0, -30, 1\n90, -30, 1\n";
  for (cvc::ariadne::DataSpec d : doc.data) {
    d.base = dir.generic_string();
    cvc::ariadne::DataStore::of(app)->bind(kPrefix, d);
  }

  SceneGraph sg(app, "ribbons");
  sg.setDiagnosticChromeVisible(false);
  put_text(app, "route.from", "0");
  put_text(app, "links.q", "0");
  std::vector<std::string> warnings;
  ari::RealizedScene rs = ari::realize_scene(sg, doc.scene, kPrefix, &warnings);
  for (const std::string &w : warnings)
    std::printf("  warning: %s\n", w.c_str());
  check(warnings.empty(), "realize: no warnings");
  std::shared_ptr<RibbonNode> track = ribbon(sg, "track"), floats = ribbon(sg, "floats"),
                              route = ribbon(sg, "route"), valued = ribbon(sg, "valued"),
                              widths = ribbon(sg, "widths"), draped = ribbon(sg, "draped"),
                              streamed = ribbon(sg, "streamed");
  std::shared_ptr<DrapedLinkNode> lnk = link(sg, "link");
  if (!check(track && floats && route && valued && widths && draped && streamed && lnk,
             "realize: every ribbon and the link are their streaming nodes"))
    return finish("cvcgl_ariadne_ribbons");

  bool tagged = false;
  for (const auto &t : rs.nodes)
    tagged = tagged || (t.id == "track" && t.tags.size() == 1 && t.tags[0] == "sim");
  check(tagged, "tags: a ribbon is in the scene's tagged nodes");

  SceneRenderer sr(sg, W, H, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  topView(sr, 0, 0, 100);
  const auto frame = [&]() {
    cvc::ariadne::sync_scene_visibility(app, rs.visibility);
    ari::tick_scene(rs, sr.renderer(), 1.0 / 60.0);
    sr.render();
  };
  frame();
  glcountInstall();

  // --- route: literal points, a window moved by a state key, no upload -------------------
  check(route->centerCount() == 3, "points: a literal route has its 3 points");
  Frame f0 = grab(sr);
  check(hueNear(f0, toPx(sr, -60, 60, 1), 2, 1), "pixels: the route draws green at x = -60");
  {
    const cvc::gl::StreamStats s0 = route->streamStats();
    const GLCount g0 = glcountRead();
    put_text(app, "route.from", "100"); // the first 100 m consumed
    frame();
    const GLCount d = glcountRead() - g0;
    const cvc::gl::StreamStats s1 = route->streamStats();
    check(s1.uploads == s0.uploads && s1.fullUploads == s0.fullUploads,
          "window: moving it uploads nothing");
    const Frame f1 = grab(sr);
    check(!hueNear(f1, toPx(sr, -60, 60, 1), 2, 1) && hueNear(f1, toPx(sr, 40, 60, 1), 2, 1),
          "window: the consumed front is gone, the rest still drawn");
    check(d.createdNothing(), "window: nothing created", glcountStr(d));
  }

  // --- track: text that grows by a point a frame --------------------------------------------
  {
    const int N = 40;
    const std::vector<float> all = track_points(N);
    int violations = 0;
    std::string first;
    for (int k = 1; k <= N; ++k) {
      const std::vector<float> head(all.begin(), all.begin() + 3 * k);
      const cvc::gl::StreamStats s0 = track->streamStats();
      const GLCount g0 = glcountRead();
      put_text(app, "tracks.a", csv(head));
      frame();
      const GLCount d = glcountRead() - g0;
      const cvc::gl::StreamStats s1 = track->streamStats();
      // One centre's two vertices for the first point; then the new centre plus the previous one
      // re-mitred: 4 vertices, 48 bytes. A growth past capacity 64 never happens at N = 40.
      const double want = k == 1 ? 24 : 48;
      const bool ok = s1.uploads - s0.uploads == 1 && s1.uploadBytes - s0.uploadBytes == want &&
                      s1.fullUploads == s0.fullUploads && d.createdNothing() &&
                      d.uploadBytes() == want;
      if (!ok && violations++ == 0)
        first = "point " + std::to_string(k) + ": " + glcountStr(d) +
                " node uploads=" + std::to_string(s1.uploads - s0.uploads) +
                " bytes=" + std::to_string(s1.uploadBytes - s0.uploadBytes) +
                " full=" + std::to_string(s1.fullUploads - s0.fullUploads);
    }
    check(violations == 0, "append: each new point uploads its quad alone (no whole-node upload)",
          first);
    check(track->centerCount() == static_cast<std::size_t>(N), "append: every point drawn");
    const Frame f = grab(sr);
    check(hueNear(f, toPx(sr, -90 + 4 * 20, -60 + 6 * std::sin(0.3 * 20), 1), 2, 0),
          "pixels: the track draws red along its points");
    check(bgNear(f, toPx(sr, -90 + 4 * 45, -60, 1), 2), "pixels: nothing past its last point");

    // The last point moves (a live position): only from it on is written again.
    std::vector<float> moved = all;
    moved[3 * (N - 1) + 1] += 5.0f;
    const cvc::gl::StreamStats s0 = track->streamStats();
    put_text(app, "tracks.a", csv(moved));
    frame();
    const cvc::gl::StreamStats s1 = track->streamStats();
    check(s1.uploads - s0.uploads == 1 && s1.uploadBytes - s0.uploadBytes == 48 &&
              s1.fullUploads == s0.fullUploads,
          "tail: a moved last point rewrites 2 centres, not the line");
    // Nothing changed: nothing read again, nothing uploaded.
    const cvc::gl::StreamStats s2 = track->streamStats();
    frame();
    check(track->streamStats().uploads == s2.uploads, "an unchanged key uploads nothing");
  }

  // --- floats: data() floats that grow, and visible: --------------------------------------
  {
    const std::vector<float> all = track_points(20);
    put_floats(app, "tracks.b", std::vector<float>(all.begin(), all.begin() + 30));
    frame();
    const cvc::gl::StreamStats s0 = floats->streamStats();
    put_floats(app, "tracks.b", std::vector<float>(all.begin(), all.begin() + 33));
    frame();
    const cvc::gl::StreamStats s1 = floats->streamStats();
    check(floats->centerCount() == 11 && s1.uploads - s0.uploads == 1 &&
              s1.uploadBytes - s0.uploadBytes == 48,
          "data(): a float vector that grew appends its new point alone");
    check(floats->isVisible(), "visible: a bound key not yet set leaves the ribbon shown");
    put_text(app, "show.floats", "0");
    frame();
    check(!floats->isVisible(), "visible: the bound key hides the ribbon");
  }

  // --- value: fields xyv + ramp ---------------------------------------------------------------
  {
    std::vector<float> v;
    for (int k = 0; k <= 30; ++k) { // x from -90 to 90 at y = 0; value 0 on the west, 1 east
      v.push_back(static_cast<float>(-90 + 6 * k));
      v.push_back(0.0f);
      v.push_back(k < 15 ? 0.0f : 1.0f);
    }
    put_text(app, "tracks.v", csv(v));
    frame();
    const Frame f = grab(sr);
    check(hueNear(f, toPx(sr, -60, 0, 1), 1, 0), "value: the west half draws the ramp's red");
    check(hueNear(f, toPx(sr, 60, 0, 1), 1, 2), "value: the east half draws the ramp's blue");
    const RibbonNode::ValueStats s0 = valued->valueStats();
    v.push_back(96.0f);
    v.push_back(0.0f);
    v.push_back(1.0f);
    put_text(app, "tracks.v", csv(v));
    frame();
    const RibbonNode::ValueStats s1 = valued->valueStats();
    check(s1.fullUploads == s0.fullUploads && s1.uploads - s0.uploads == 1 &&
              s1.bytes - s0.bytes == sizeof(float) && s1.uploadCalls - s0.uploadCalls == 1,
          "value: an appended point uploads its one value texel");
  }

  // --- width: fields xyzw --------------------------------------------------------------------
  {
    put_text(app, "tracks.w", "-80,-90,1,2, 0,-90,1,2, 0,-90,1,20, 80,-90,1,20");
    frame();
    check(std::fabs(widths->centerHalfWidth(0) - 1.0f) < 1e-6f &&
              std::fabs(widths->centerHalfWidth(3) - 10.0f) < 1e-6f,
          "width: each point's w is its full width");
    const Frame f = grab(sr);
    check(hueNear(f, toPx(sr, 40, -84, 1), 1, 0) ||
              !isBg(f.at(int(toPx(sr, 40, -84, 1)[0]), int(toPx(sr, 40, -84, 1)[1]))),
          "width: the wide end draws 6 m off its line");
    check(isBg(f.at(int(toPx(sr, -40, -84, 1)[0]), int(toPx(sr, -40, -84, 1)[1]))),
          "width: the narrow end does not");
  }

  // --- draped: z from the ground -------------------------------------------------------------
  {
    put_text(app, "tracks.d", "-20,30, 0,30, 20,30");
    frame();
    const std::vector<float> c = draped->centerVertices(1);
    const double h = rs.height_fields.at("ground")->height(0.0, 30.0);
    check(std::fabs(c[2] - (h + 0.5)) < 1e-3, "draped: a point sits on the ground plus its lift",
          "z " + std::to_string(c[2]) + " vs " + std::to_string(h + 0.5));
  }

  // --- dataset: source: { data, item } ----------------------------------------------------
  {
    std::shared_ptr<RibbonNode> dataset = ribbon(sg, "dataset");
    check(cvc::ariadne::DataStore::of(app)->wait(key("data.route_csv"), 30.0),
          "dataset: the text dataset loads");
    frame();
    check(dataset && dataset->centerCount() == 3, "dataset: its numbers are the ribbon's points");
    const Frame f = grab(sr);
    check(!isBg(f.at(int(toPx(sr, 40, -30, 1)[0]), int(toPx(sr, 40, -30, 1)[1]))),
          "dataset: the ribbon draws");
  }

  // --- stream: each frame's points appended ------------------------------------------------
  if (pts) {
    publish_points(*pts, {-90, 90, 1, -60, 90, 1});
    frame();
    const cvc::gl::StreamStats s0 = streamed->streamStats();
    publish_points(*pts, {-30, 90, 1});
    frame();
    const cvc::gl::StreamStats s1 = streamed->streamStats();
    check(streamed->centerCount() == 3 && s1.uploads - s0.uploads == 1 &&
              s1.uploadBytes - s0.uploadBytes == 48,
          "stream: a frame's point appends alone",
          "count " + std::to_string(streamed->centerCount()));
  }

  // --- link: between two nodes, on the ground, moved and recoloured with no upload ----------
  {
    double mid[3];
    lnk->centerAt(0.5, mid);
    const double h = lnk->heightField()->sample(0.0, 0.0);
    check(std::fabs(mid[0]) < 1e-6 && std::fabs(mid[2] - (h + 1.0)) < 1e-4,
          "link: its middle hangs over the dome's top, at its lift");
    Frame f = grab(sr);
    check(hueNear(f, toPx(sr, -20, 0, 0), 2, 0), "link: value 0 draws the ramp's red");
    const cvc::gl::StreamStats s0 = lnk->streamStats();
    const GLCount g0 = glcountRead();
    put_text(app, "links.q", "1");
    sg.getGraphics("car_a")->setPosition(-40, 40, 0);
    frame();
    const GLCount d = glcountRead() - g0;
    const cvc::gl::StreamStats s1 = lnk->streamStats();
    check(s1.uploads == s0.uploads && s1.fullUploads == s0.fullUploads && d.createdNothing(),
          "link: moved and recoloured with no upload", glcountStr(d));
    double a[3];
    lnk->centerAt(0.0, a);
    check(std::fabs(a[0] + 40) < 1e-4 && std::fabs(a[1] - 40) < 1e-4,
          "link: its end follows the node");
    f = grab(sr);
    check(hueNear(f, toPx(sr, 0, 20, 0), 3, 1), "link: value 1 draws the ramp's green");
  }

  // --- regen: the ground made again re-drapes what stands on it ---------------------------
  {
    auto field = std::const_pointer_cast<cvc::height_field>(rs.height_fields.at("ground"));
    std::vector<float> raised = field->heights();
    for (float &v : raised)
      v += 10.0f;
    *field = cvc::height_field(field->rows(), field->cols(), field->x0(), field->y0(), field->dx(),
                               field->dy(), raised);
    frame();
    const std::vector<float> c = draped->centerVertices(1);
    check(std::fabs(c[2] - (field->height(0.0, 30.0) + 0.5)) < 1e-3,
          "regen: a draped ribbon sits on the new ground");
    double mid[3];
    lnk->centerAt(0.5, mid);
    check(std::fabs(mid[2] - (lnk->heightField()->sample(mid[0], mid[1]) + 1.0)) < 1e-4 &&
              lnk->heightField()->sample(0.0, 0.0) > 25.0,
          "regen: the link drapes over the new heights");
  }

  // --- regen on a new grid: the link takes a new texture ----------------------------------
  {
    auto field = std::const_pointer_cast<cvc::height_field>(rs.height_fields.at("ground"));
    const std::shared_ptr<cvc::gl::HeightFieldTexture> before = lnk->heightField();
    std::vector<float> flat(21 * 21, 3.0f);
    *field = cvc::height_field(21, 21, -100.0, -100.0, 10.0, 10.0, flat);
    frame();
    double mid[3];
    lnk->centerAt(0.5, mid);
    check(lnk->heightField() != before && lnk->heightField()->nx() == 21 &&
              std::fabs(mid[2] - 4.0) < 1e-4,
          "regen: a ground on a new grid gives the link a new texture");
    const Frame f = grab(sr);
    check(hueNear(f, toPx(sr, 0, 20, 0), 3, 1), "regen: the re-draped link still draws");
  }

  testValuesDirect(app, cvc::gl::StreamingMapperKind::Classic);
  testValuesDirect(app, cvc::gl::StreamingMapperKind::LowMemory);

  check(ErrorCounter::errors() == 0, "no VTK errors (shaders compiled)");
  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
  return finish("cvcgl_ariadne_ribbons");
}
