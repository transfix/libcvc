/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcgl_streaming_overlays -- the three streaming overlay shapes, on BOTH mapper
// families (classic, and the low-memory one forced), rendered for real:
//
//   track   RibbonNode::append: each append uploads ONLY the new quad (48 B,
//           one GL call, nothing created), counted by the node and at the driver;
//           today's GeometryNode::updateVertices path is measured alongside.
//   spine   RibbonNode::assign = one sub-range upload per replan; per-frame
//           setVisibleCenters (draw range + fractional clip) = 0 uploads; a
//           shorter replan's stale tail is hidden, never rewritten.
//   link    DrapedLinkNode: draped onto a HeightFieldTexture in the vertex
//           shader, endpoint moves = 0 uploads, culled without reserved bounds,
//           updateRows = one row-band texture upload.
//   parity  each one against the SAME triangles drawn by a plain GeometryNode.
//   shadows all three under vtkShadowMapPass: one upload per frame, not per
//           pass, and no shader errors.
//
// Skips (rc 0) where nothing rasterises unless CVC_REQUIRE_RENDER=1. Synthetic
// geometry only.

#include "streaming_test_util.h"

#include <cmath>
#include <cvc/gl/DrapedLinkNode.h>
#include <cvc/gl/HeightFieldTexture.h>
#include <cvc/gl/RibbonNode.h>
#include <memory>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkPlane.h>
#include <vtkPlaneCollection.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>

using namespace cvcgl_test;
using cvc::gl::DrapedLinkNode;
using cvc::gl::GeometryNode;
using cvc::gl::HeightFieldTexture;
using cvc::gl::RibbonNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::StreamingMapperKind;

namespace {

constexpr double C = 500.0; // scene centre: well away from the origin
const cvc::bounding_box kBounds(C - 100, C - 100, -1, C + 100, C + 100, 45);
constexpr int W = 200, H = 200;

std::array<double, 2> trackPath(int k) { return {C - 90 + 2.5 * k, C + 25 * std::sin(k * 0.07)}; }
std::array<double, 2> routeA(int k) { return {C - 90 + 2.8 * k, C - 40 + 0.6 * k}; }
std::array<double, 2> routeB(int k) { return {C - 80 + 3.0 * k, C + 60}; }

// The triangles a ribbon draws, as a plain mesh: centres [first, last] of
// `rib`, the first pair optionally cut at a fractional position `f` into
// segment `first` (what the fragment clip does).
cvc::geometry ribbonTwin(cvc::app &app, const RibbonNode &rib, std::size_t first, std::size_t last,
                         double f = 0.0) {
  cvc::geometry g(app);
  for (std::size_t k = first; k <= last; ++k) {
    std::vector<float> v = rib.centerVertices(k);
    if (k == first && f > 0.0) {
      const std::vector<float> nx = rib.centerVertices(k + 1);
      for (int i = 0; i < 6; ++i)
        v[i] = static_cast<float>(v[i] + (nx[i] - v[i]) * f);
    }
    g.points().push_back({v[0], v[1], v[2]});
    g.points().push_back({v[3], v[4], v[5]});
    g.normals().push_back({0.0, 0.0, 1.0});
    g.normals().push_back({0.0, 0.0, 1.0});
  }
  for (std::size_t s = 0; s + first < last; ++s) {
    const cvc::geometry::index_t l0 = 2 * s, r0 = l0 + 1, l1 = l0 + 2, r1 = l0 + 3;
    g.tris().push_back({l0, r0, r1});
    g.tris().push_back({l0, r1, l1});
  }
  g.set_geometry_type(cvc::geometry::SURFACE_TRI);
  return g;
}

// Draw `a` alone, then `b` alone, in the same view; return the pixel diff.
// Toggles the actors' own visibility: GraphicsNode::setVisible removes the
// prop from the renderer, which releases its GL buffers, so a re-shown node
// re-uploads in full -- that would pollute the upload counts that follow.
void showProp(cvc::gl::GraphicsNode &n, bool on) { n.prop()->SetVisibility(on ? 1 : 0); }
long parity(SceneRenderer &sr, cvc::gl::GraphicsNode &a, cvc::gl::GraphicsNode &b, long &lit) {
  showProp(a, true);
  showProp(b, false);
  const Frame fa = grab(sr);
  showProp(a, false);
  showProp(b, true);
  const Frame fb = grab(sr);
  showProp(b, false);
  showProp(a, true);
  lit = litPixels(fa);
  return diffPixels(fa, fb);
}

// ── track ───────────────────────────────────────────────────────────────────
void testTrack(cvc::app &app, StreamingMapperKind kind) {
  std::printf("track: RibbonNode::append [%s]\n", kindName(kind));
  SceneGraph sg(app, std::string("track_") + kindName(kind));
  sg.setDiagnosticChromeVisible(false);
  auto track =
      sg.getGraphicsRoot()->addGraphicsChild<RibbonNode>("track", 1024, 1.5f, kBounds, kind);
  flat(*track, 1.0, 0.1, 0.1);
  SceneRenderer sr(sg, W, H, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  topView(sr, C, C, 100);
  sr.render();
  glcountInstall();

  int violations = 0;
  std::string firstViolation;
  double bytes = 0;
  const int N = 60;
  for (int k = 0; k < N; ++k) {
    const auto p = trackPath(k);
    const cvc::gl::StreamStats s0 = track->streamStats();
    const GLCount g0 = glcountRead();
    track->append(static_cast<float>(p[0]), static_cast<float>(p[1]), 0.5f);
    sr.render();
    const GLCount d = glcountRead() - g0;
    const cvc::gl::StreamStats s1 = track->streamStats();
    const double want = k == 0 ? 24 : 48;
    bytes += d.uploadBytes();
    const bool ok = d.createdNothing() && d.uploadCalls() == 1 && d.uploadBytes() == want &&
                    s1.uploads - s0.uploads == 1 && s1.uploadBytes - s0.uploadBytes == want &&
                    s1.fullUploads == s0.fullUploads;
    if (!ok && violations++ == 0)
      firstViolation = "append " + std::to_string(k) + ": " + glcountStr(d);
  }
  check(violations == 0, "60 appends: each 1 sub-upload of 48 B (24 B first), nothing created",
        violations ? firstViolation : "");
  std::printf("  measured: %.1f B per append through the streaming path\n", bytes / N);

  const Frame f = grab(sr);
  const auto mid = trackPath(30), last = trackPath(N - 1), beyond = trackPath(70);
  check(hueNear(f, toPx(sr, mid[0], mid[1], 0.5), 2, 0), "history drawn");
  check(hueNear(f, toPx(sr, last[0] - 1.0, last[1], 0.5), 2, 0), "latest segment drawn");
  check(bgNear(f, toPx(sr, beyond[0], beyond[1], 0.5), 3), "unappended capacity not drawn");
  check(bgNear(f, toPx(sr, C - 99, C - 99, 0), 3), "no stray triangles toward unwritten points");

  // Parity: the same triangles through a plain GeometryNode.
  auto twin = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("twin");
  twin->setGeometry(ribbonTwin(app, *track, 0, N - 1));
  flat(*twin, 1.0, 0.1, 0.1);
  long lit = 0;
  const long diff = parity(sr, *track, *twin, lit);
  // The floor is the ribbon's own area in pixels (width x arc length at
  // topView's H / 200 px per unit; 498 px here), with slack: a fixed 500 sat
  // ABOVE that area and only passed where the rasteriser adds edge pixels
  // (Linux, 584) -- Apple's software renderer lights exactly 498.
  const double ppu = H / (2.0 * 100.0);
  const double area = 2.0 * track->halfWidth() * track->arcLength() * ppu * ppu;
  check(lit > 0.8 * area && diff >= 0 && diff <= lit / 100,
        "pixels match the wholesale GeometryNode render",
        std::to_string(diff) + " of " + std::to_string(lit) + " lit pixels differ (area " +
            std::to_string(static_cast<long>(area)) + " px)");

  if (kind != StreamingMapperKind::Classic)
    return;
  // Today's path, measured for comparison: a 2048-point GeometryNode re-sent
  // whole by updateVertices on every append (tail collapsed onto the head).
  auto old = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("wholesale");
  {
    cvc::geometry g = ribbonTwin(app, *track, 0, N - 1);
    while (g.points().size() < 2048) {
      g.points().push_back(g.points().back());
      g.normals().push_back({0.0, 0.0, 1.0});
    }
    old->setGeometry(g);
  }
  flat(*old, 1.0, 0.1, 0.1);
  showProp(*track, false);
  showProp(*twin, false);
  sr.render();
  const GLCount g0 = glcountRead();
  std::vector<double> xyz(3 * 2048);
  for (int it = 0; it < 12; ++it) {
    for (int i = 0; i < 2048; ++i) {
      const auto p = trackPath(std::min(i / 2, N - 1));
      xyz[3 * i] = p[0] + (i % 2 ? 1.0 : -1.0) * 0.001 * it;
      xyz[3 * i + 1] = p[1];
      xyz[3 * i + 2] = 0.5;
    }
    old->updateVertices(xyz);
    sr.render();
  }
  const GLCount d = glcountRead() - g0;
  std::printf("  measured: today's updateVertices path: %.0f B per append in %.1f upload calls, "
              "%.1f buffers + %.1f textures created per append\n",
              d.uploadBytes() / 12, d.uploadCalls() / 12, d.v[K_GENBUF] / 12, d.v[K_GENTEX] / 12);
  check(d.uploadBytes() / 12 > 100 * 48, "the streaming append is >100x smaller than a rewrite");
}

// ── spine ───────────────────────────────────────────────────────────────────
void testSpine(cvc::app &app, StreamingMapperKind kind) {
  std::printf("spine: RibbonNode::assign + setVisibleCenters [%s]\n", kindName(kind));
  SceneGraph sg(app, std::string("spine_") + kindName(kind));
  sg.setDiagnosticChromeVisible(false);
  auto spine =
      sg.getGraphicsRoot()->addGraphicsChild<RibbonNode>("spine", 256, 1.5f, kBounds, kind);
  flat(*spine, 0.1, 1.0, 0.1);
  SceneRenderer sr(sg, W, H, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  topView(sr, C, C, 100);
  sr.render();
  glcountInstall();

  auto assignRoute = [&](std::array<double, 2> (*route)(int), int n) {
    std::vector<float> xyz;
    for (int k = 0; k < n; ++k) {
      const auto p = route(k);
      xyz.insert(xyz.end(), {static_cast<float>(p[0]), static_cast<float>(p[1]), 0.5f});
    }
    spine->assign(xyz.data(), static_cast<std::size_t>(n));
  };

  GLCount g0 = glcountRead();
  cvc::gl::StreamStats s0 = spine->streamStats();
  assignRoute(routeA, 65);
  sr.render();
  GLCount d = glcountRead() - g0;
  cvc::gl::StreamStats s1 = spine->streamStats();
  check(d.createdNothing() && d.uploadCalls() == 1 && d.uploadBytes() == 130 * 12 &&
            s1.uploads - s0.uploads == 1 && s1.uploadBytes - s0.uploadBytes == 1560,
        "replan (65 centres) = one sub-range upload of 1560 B", glcountStr(d));

  int violations = 0;
  for (int fr = 1; fr <= 20; ++fr) {
    g0 = glcountRead();
    spine->setVisibleCenters(fr + 0.5); // vehicle progress, fractional centre index
    sr.render();
    d = glcountRead() - g0;
    violations += d.uploadCalls() != 0 || !d.createdNothing();
  }
  check(violations == 0, "20 per-frame visible-range moves: 0 uploads");
  Frame f = grab(sr);
  const auto a10 = routeA(10), a19 = routeA(19), a22 = routeA(22), a55 = routeA(55);
  check(bgNear(f, toPx(sr, a10[0], a10[1], 0.5), 2) && bgNear(f, toPx(sr, a19[0], a19[1], 0.5), 2),
        "consumed front hidden");
  check(hueNear(f, toPx(sr, a22[0], a22[1], 0.5), 2, 1) &&
            hueNear(f, toPx(sr, a55[0], a55[1], 0.5), 2, 1),
        "remaining route drawn");

  // Parity, including the fractional cut at 20.5.
  auto twin = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("twin");
  twin->setGeometry(ribbonTwin(app, *spine, 20, 64, 0.5));
  flat(*twin, 0.1, 1.0, 0.1);
  long lit = 0;
  const long diff = parity(sr, *spine, *twin, lit);
  check(lit > 200 && diff >= 0 && diff <= lit / 50,
        "clipped spine matches the wholesale render of the cut route",
        std::to_string(diff) + " of " + std::to_string(lit) + " lit pixels differ");

  // A shorter replan: one upload, the old route's tail hidden, not rewritten.
  g0 = glcountRead();
  s0 = spine->streamStats();
  assignRoute(routeB, 50);
  f = grab(sr);
  d = glcountRead() - g0;
  s1 = spine->streamStats();
  check(d.uploadCalls() == 1 && d.uploadBytes() == 100 * 12 && s1.uploads - s0.uploads == 1,
        "replan (50 centres) = one sub-range upload of 1200 B", glcountStr(d));
  const auto a60 = routeA(60), b10 = routeB(10);
  check(bgNear(f, toPx(sr, a60[0], a60[1], 0.5), 2), "stale tail of the longer route hidden");
  check(hueNear(f, toPx(sr, b10[0], b10[1], 0.5), 2, 1), "new route drawn from its start");

  g0 = glcountRead();
  spine->setVisibleCenters(5.0, 30.25);
  f = grab(sr);
  d = glcountRead() - g0;
  const auto b2 = routeB(2), b20 = routeB(20), b40 = routeB(40);
  check(d.uploadCalls() == 0 && bgNear(f, toPx(sr, b2[0], b2[1], 0.5), 2) &&
            hueNear(f, toPx(sr, b20[0], b20[1], 0.5), 2, 1) &&
            bgNear(f, toPx(sr, b40[0], b40[1], 0.5), 2),
        "a [5, 30.25] window: both ends cut, 0 uploads");
  g0 = glcountRead();
  spine->clearCenters();
  f = grab(sr);
  d = glcountRead() - g0;
  check(d.uploadCalls() == 0 && litPixels(f) == 0, "clearCenters: nothing drawn, 0 uploads");
}

// ── draped link ─────────────────────────────────────────────────────────────
double bump(double x, double y) {
  const double dx = x - C, dy = y - C;
  return 20.0 * std::exp(-(dx * dx + dy * dy) / (2.0 * 20.0 * 20.0));
}

std::shared_ptr<HeightFieldTexture> bumpField(int n) {
  const double d = 200.0 / (n - 1);
  auto hf = std::make_shared<HeightFieldTexture>(n, n, C - 100, C - 100, d, d);
  std::vector<float> h(static_cast<std::size_t>(n) * n);
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i)
      h[static_cast<std::size_t>(j) * n + i] =
          static_cast<float>(bump(C - 100 + i * d, C - 100 + j * d));
  hf->setHeights(h);
  return hf;
}

void obliqueView(SceneRenderer &sr) { // looking +y, 35 degrees down
  const double el = 35.0 * 3.14159265358979323846 / 180.0, D = 600;
  sr.setCamera(C, C - D * std::cos(el), 10 + D * std::sin(el), C, C, 10, 0, std::sin(el),
               std::cos(el), 30.0, 1.0, 2000.0);
  vtkCamera *cam = sr.renderer()->GetActiveCamera();
  cam->ParallelProjectionOn();
  cam->SetParallelScale(60);
}

// The link's vertices as the vertex shader places them, for a wholesale twin.
cvc::geometry linkTwin(cvc::app &app, const HeightFieldTexture &hf, double ax, double ay, double bx,
                       double by, int stations, double half, double lift) {
  cvc::geometry g(app);
  const double len = std::hypot(bx - ax, by - ay);
  const double dx = (bx - ax) / len, dy = (by - ay) / len;
  for (int k = 0; k < stations; ++k) {
    const double t = static_cast<double>(k) / (stations - 1);
    for (double side : {1.0, -1.0}) {
      const double x = ax + (bx - ax) * t - dy * side * half;
      const double y = ay + (by - ay) * t + dx * side * half;
      g.points().push_back({x, y, hf.sample(x, y) + lift});
      g.normals().push_back({0.0, 0.0, 1.0});
    }
  }
  for (int s = 0; s + 1 < stations; ++s) {
    const cvc::geometry::index_t l0 = 2 * s, r0 = l0 + 1, l1 = l0 + 2, r1 = l0 + 3;
    g.tris().push_back({l0, r0, r1});
    g.tris().push_back({l0, r1, l1});
  }
  g.set_geometry_type(cvc::geometry::SURFACE_TRI);
  return g;
}

void testLink(cvc::app &app, StreamingMapperKind kind) {
  std::printf("link: DrapedLinkNode [%s]\n", kindName(kind));
  SceneGraph sg(app, std::string("link_") + kindName(kind));
  sg.setDiagnosticChromeVisible(false);
  auto hf = bumpField(128);
  auto link = sg.getGraphicsRoot()->addGraphicsChild<DrapedLinkNode>("link", hf, 24, kind);
  link->setStyle(2.0f, 1.0f, 0.15f, 0.35f, 1.0f);
  link->setEndpoints(static_cast<float>(C - 50), static_cast<float>(C), static_cast<float>(C + 50),
                     static_cast<float>(C));
  SceneRenderer sr(sg, W, H, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  obliqueView(sr);
  Frame f = grab(sr);
  glcountInstall();

  auto onLink = [&](const Frame &fr, double x) {
    double p[3];
    link->centerAt((x - (C - 50)) / 100.0, p);
    return hueNear(fr, toPx(sr, p[0], p[1], p[2]), 3, 2);
  };
  std::string hits;
  bool all = true;
  for (double x : {C - 40, C - 20, C, C + 20, C + 40}) {
    const bool h = onLink(f, x);
    all = all && h;
    hits += h ? "y" : "n";
  }
  check(all, "drawn along the draped centre line (centerAt)", "stations hit: " + hits);
  check(bgNear(f, toPx(sr, C, C, 1.0), 2), "not flat: nothing where an undraped link would be");
  check(sr.renderer()->GetNumberOfPropsRendered() == 1, "drawn (reserved bounds) -- 1 prop");

  // Without bounds that hold it, VTK culls the template before the shader runs.
  link->setReservedBounds(cvc::bounding_box(C + 5000, C + 5000, 0, C + 5001, C + 5001, 1));
  f = grab(sr);
  check(sr.renderer()->GetNumberOfPropsRendered() == 0 && !onLink(f, C),
        "with bounds that miss the view it is culled");
  link->refreshReservedBounds();
  f = grab(sr);
  check(sr.renderer()->GetNumberOfPropsRendered() == 1 && onLink(f, C),
        "refreshReservedBounds restores it");

  // Parity against the CPU-draped mesh.
  auto twin = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("twin");
  twin->setGeometry(linkTwin(app, *hf, C - 50, C, C + 50, C, 24, 2.0, 1.0));
  flat(*twin, 0.15, 0.35, 1.0);
  long lit = 0;
  const long diff = parity(sr, *link, *twin, lit);
  check(lit > 300 && diff >= 0 && diff <= lit / 50,
        "pixels match the wholesale render of the CPU-draped mesh",
        std::to_string(diff) + " of " + std::to_string(lit) + " lit pixels differ");

  // Endpoint moves: uniforms only.
  int violations = 0;
  double y = C;
  for (int fr = 1; fr <= 20; ++fr) {
    y = C + 2.0 * fr;
    const GLCount g0 = glcountRead();
    link->setEndpoints(static_cast<float>(C - 50), static_cast<float>(y),
                       static_cast<float>(C + 50), static_cast<float>(y));
    sr.render();
    const GLCount d = glcountRead() - g0;
    violations += d.uploadCalls() != 0 || !d.createdNothing();
  }
  check(violations == 0, "20 endpoint moves: 0 uploads");
  f = grab(sr);
  check(onLink(f, C) && onLink(f, C - 30), "moved with the uniforms");

  // Patching terrain rows: one row-band texture upload, and the link follows.
  const HeightFieldTexture::Stats h0 = hf->stats();
  const GLCount g0 = glcountRead();
  const int row = static_cast<int>(std::lround((y - (C - 100)) / hf->dy()));
  std::vector<float> rows(static_cast<std::size_t>(5) * hf->nx(), 30.0f);
  hf->updateRows(row - 2, 5, rows.data());
  f = grab(sr);
  const GLCount d = glcountRead() - g0;
  const HeightFieldTexture::Stats h1 = hf->stats();
  check(d.v[K_TEXSUB] == 1 && d.v[K_TEXSUB_B] == 5 * hf->nx() * 4 && d.v[K_TEXIMG] == 0 &&
            h1.rowUploads - h0.rowUploads == 1 && h1.fullUploads == h0.fullUploads,
        "updateRows(5 rows) = one 2560-byte glTexSubImage2D", glcountStr(d));
  check(onLink(f, C - 30), "the link follows the patched terrain");

  // The public shader-replacement API reaches the GPU after the first draw on
  // BOTH mappers (VTK's low-memory one ignores its shader property once its
  // program is built) -- and cannot take the drape away.
  const int errors0 = ErrorCounter::errors();
  link->addFragmentShaderReplacement("//VTK::UniformFlow::Impl",
                                     "//VTK::UniformFlow::Impl\n  discard;\n");
  f = grab(sr);
  check(!onLink(f, C) && !onLink(f, C - 30),
        "a caller replacement added after the first draw takes effect (discards it all)");
  link->clearShaderReplacements();
  link->addVertexShaderReplacement("//VTK::Clip::Impl", "//VTK::Clip::Impl\n  // caller code\n");
  f = grab(sr);
  check(onLink(f, C) && onLink(f, C - 30) && ErrorCounter::errors() == errors0,
        "cleared again: drawn, and still draped, with a caller replacement on the drape's anchor");
}

// The link under clipping planes (what a parent's setClipChildren hands its
// children through applyClipPlanes): VTK's clip distances must use the draped
// point, not the template's (t, side, 0) near the origin. The planes are set on
// the mapper directly, oriented the way VTK keeps geometry (normals INTO the
// box), so the check is about the drape alone.
vtkSmartPointer<vtkPlaneCollection> inwardBox(const cvc::bounding_box &b) {
  auto pc = vtkSmartPointer<vtkPlaneCollection>::New();
  const double o[6][3] = {{b.maxx, 0, 0}, {b.minx, 0, 0}, {0, b.maxy, 0},
                          {0, b.miny, 0}, {0, 0, b.maxz}, {0, 0, b.minz}};
  const double n[6][3] = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};
  for (int i = 0; i < 6; ++i) {
    auto plane = vtkSmartPointer<vtkPlane>::New();
    plane->SetOrigin(o[i][0], o[i][1], o[i][2]);
    plane->SetNormal(n[i][0], n[i][1], n[i][2]);
    pc->AddItem(plane);
  }
  return pc;
}

void testLinkClip(cvc::app &app, StreamingMapperKind kind) {
  std::printf("link clip: DrapedLinkNode under clipping planes [%s]\n", kindName(kind));
  for (const bool boxHoldsLink : {true, false}) {
    // VTK's low-memory mapper has no clip planes at all: only "not wrongly
    // clipped" is meaningful there.
    if (!boxHoldsLink && kind != StreamingMapperKind::Classic)
      continue;
    SceneGraph sg(app, std::string("clip_") + kindName(kind) + (boxHoldsLink ? "_in" : "_out"));
    sg.setDiagnosticChromeVisible(false);
    auto hf = bumpField(64);
    auto link = sg.getGraphicsRoot()->addGraphicsChild<DrapedLinkNode>("link", hf, 24, kind);
    link->setStyle(2.0f, 1.0f, 0.15f, 0.35f, 1.0f);
    link->setEndpoints(static_cast<float>(C - 50), static_cast<float>(C),
                       static_cast<float>(C + 50), static_cast<float>(C));
    // In: a box around the drawn link (not the origin). Out: a box around the
    // template only.
    const cvc::bounding_box box =
        boxHoldsLink ? cvc::bounding_box(C - 100, C - 100, -50, C + 100, C + 100, 80)
                     : cvc::bounding_box(-5, -5, -5, 5, 5, 5);
    vtkActor::SafeDownCast(link->prop())->GetMapper()->SetClippingPlanes(inwardBox(box));
    SceneRenderer sr(sg, W, H, /*offscreen=*/true);
    sr.setBackground(0, 0, 0);
    obliqueView(sr);
    const Frame f = grab(sr);
    int hits = 0;
    for (double t : {0.1, 0.3, 0.5, 0.7, 0.9}) {
      double p[3];
      link->centerAt(t, p);
      hits += hueNear(f, toPx(sr, p[0], p[1], p[2]), 3, 2) ? 1 : 0;
    }
    if (boxHoldsLink)
      check(hits == 5, "a link inside the clip box is drawn", std::to_string(hits) + "/5 stations");
    else
      check(hits == 0 && litPixels(f) == 0,
            "a link outside the clip box is clipped, though its template is inside",
            std::to_string(hits) + "/5 stations");
  }
}

// Terrain that arrives AFTER the link (heights changed under it): the derived
// bounds follow, or VTK culls the link by its stale box.
void testLinkTerrainLater(cvc::app &app, StreamingMapperKind kind) {
  std::printf("link bounds follow the height field [%s]\n", kindName(kind));
  SceneGraph sg(app, std::string("later_") + kindName(kind));
  sg.setDiagnosticChromeVisible(false);
  const int n = 32;
  const double d = 200.0 / (n - 1);
  auto hf = std::make_shared<HeightFieldTexture>(n, n, C - 100, C - 100, d, d); // flat, z = 0
  auto link = sg.getGraphicsRoot()->addGraphicsChild<DrapedLinkNode>("link", hf, 24, kind);
  link->setStyle(3.0f, 1.0f, 0.15f, 0.35f, 1.0f);
  link->setEndpoints(static_cast<float>(C - 50), static_cast<float>(C), static_cast<float>(C + 50),
                     static_cast<float>(C));
  SceneRenderer sr(sg, W, H, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  // Looking down from z = 500 with a far plane at 250: z 250..499 is in view,
  // the link's flat-terrain box (z ~0) is not.
  sr.setCamera(C, C, 500, C, C, 0, 0, 1, 0, 30.0, 1.0, 250.0);
  sr.renderer()->GetActiveCamera()->ParallelProjectionOn();
  sr.renderer()->GetActiveCamera()->SetParallelScale(100);
  sr.render();
  hf->setHeights(std::vector<float>(static_cast<std::size_t>(n) * n, 300.0f));
  const Frame f = grab(sr);
  double p[3];
  link->centerAt(0.5, p);
  check(sr.renderer()->GetNumberOfPropsRendered() == 1 &&
            hueNear(f, toPx(sr, p[0], p[1], p[2]), 3, 2),
        "drawn at its new height, not culled by the flat-terrain box",
        "z " + std::to_string(p[2]) + ", maxz " + std::to_string(link->reservedBounds().maxz));
}

// A track that drives out of its reserved box stays drawn.
void testTrackLeavesBox(cvc::app &app, StreamingMapperKind kind) {
  std::printf("track leaves its box [%s]\n", kindName(kind));
  SceneGraph sg(app, std::string("leave_") + kindName(kind));
  sg.setDiagnosticChromeVisible(false);
  auto track = sg.getGraphicsRoot()->addGraphicsChild<RibbonNode>(
      "track", 8, 1.5f, cvc::bounding_box(C - 10, C - 10, -1, C + 10, C + 10, 1), kind);
  flat(*track, 1.0, 0.1, 0.1);
  for (int k = 0; k < 30; ++k)
    track->append(static_cast<float>(C + 10 * k), static_cast<float>(C), 0.5f);
  SceneRenderer sr(sg, W, H, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  topView(sr, C + 250, C, 40); // the original box is well off-screen
  const Frame f = grab(sr);
  check(sr.renderer()->GetNumberOfPropsRendered() == 1 &&
            hueNear(f, toPx(sr, C + 255, C, 0.5), 2, 0),
        "the far end is drawn (the box grew with it), not frustum-culled");
}

// VTK's CPU picker hits the polydata, not what the shader draws: by default
// the streaming overlays are not pickable at all.
void testPicking(cvc::app &app, StreamingMapperKind kind) {
  std::printf("picking [%s]\n", kindName(kind));
  SceneGraph sg(app, std::string("pick_") + kindName(kind));
  sg.setDiagnosticChromeVisible(false);
  // A flat field whose extent CONTAINS the origin, so the link's reserved box
  // does too and VTK's bounds pre-filter lets a pick there reach the template
  // cells at (t, +-1, 0). The drawn link is far from them.
  auto hf = std::make_shared<HeightFieldTexture>(32, 32, -100.0, -100.0, 200.0 / 31, 200.0 / 31);
  auto link = sg.getGraphicsRoot()->addGraphicsChild<DrapedLinkNode>("link", hf, 12, kind);
  link->setEndpoints(40.0f, 40.0f, 80.0f, 40.0f);
  auto route = sg.getGraphicsRoot()->addGraphicsChild<RibbonNode>("route", 32, 1.5f, kBounds, kind);
  flat(*route, 1.0, 0.1, 0.1);
  {
    std::vector<float> xyz;
    for (int k = 0; k < 10; ++k)
      xyz.insert(xyz.end(),
                 {static_cast<float>(C - 90 + 10 * k), static_cast<float>(C - 60), 0.5f});
    route->assign(xyz.data(), 10);
    route->assign(xyz.data(), 3); // replan shorter: centres 3..9 are a hidden tail
  }
  SceneRenderer sr(sg, W, H, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  check(!link->pickable() && !route->pickable(), "not pickable by default");
  double w[3];
  topView(sr, 0.5, 0.0, 10); // where the link's undraped template sits
  sr.render();
  const auto t = toPx(sr, 0.5, 0.0, 0.0);
  check(!sr.pickWorld(t[0], t[1], w), "nothing picked at the link's invisible template");
  link->setPickable(true); // what every streaming node did before
  sr.render();
  check(sr.pickWorld(t[0], t[1], w) && std::fabs(w[0] - 0.5) < 0.1,
        "... which a pickable link WOULD report (the check can fail)");
  link->setPickable(false);
  topView(sr, C, C, 100);
  sr.render();
  const auto tail = toPx(sr, C - 90 + 60, C - 60, 0.5);
  check(!sr.pickWorld(tail[0], tail[1], w), "nothing picked on the route's hidden tail");
  route->setPickable(true);
  sr.render();
  const auto shown = toPx(sr, C - 90 + 10, C - 60, 0.5);
  check(route->pickable() && sr.pickWorld(shown[0], shown[1], w) &&
            std::fabs(w[0] - (C - 80)) < 2.0,
        "setPickable(true) opts a node in");
}

// ── all of it under shadows ─────────────────────────────────────────────────
void testShadows(cvc::app &app, StreamingMapperKind kind) {
  std::printf("shadows: all three under vtkShadowMapPass [%s]\n", kindName(kind));
  SceneGraph sg(app, std::string("shadow_") + kindName(kind));
  sg.setDiagnosticChromeVisible(false);
  const int errors0 = ErrorCounter::errors();
  auto track =
      sg.getGraphicsRoot()->addGraphicsChild<RibbonNode>("track", 512, 1.5f, kBounds, kind);
  flat(*track, 1.0, 0.1, 0.1);
  for (int k = 0; k < 40; ++k)
    track->append(static_cast<float>(C - 80 + 3 * k), static_cast<float>(C - 30), 0.5f);
  track->setVisibleCenters(5.0); // a true sub-range under the shadow passes too
  auto spine =
      sg.getGraphicsRoot()->addGraphicsChild<RibbonNode>("spine", 128, 1.5f, kBounds, kind);
  flat(*spine, 0.1, 1.0, 0.1);
  {
    std::vector<float> xyz;
    for (int k = 0; k < 40; ++k)
      xyz.insert(xyz.end(), {static_cast<float>(C - 80 + 3 * k), static_cast<float>(C + 30), 0.5f});
    spine->assign(xyz.data(), 40);
  }
  auto hf = bumpField(64);
  auto link = sg.getGraphicsRoot()->addGraphicsChild<DrapedLinkNode>("link", hf, 24, kind);
  link->setStyle(2.0f, 1.0f, 0.15f, 0.35f, 1.0f);
  link->setEndpoints(static_cast<float>(C - 60), static_cast<float>(C + 70),
                     static_cast<float>(C + 60), static_cast<float>(C + 70));
  cvc::geometry ground(app);
  for (double gx : {C - 100, C + 100})
    for (double gy : {C - 100, C + 100}) {
      ground.points().push_back({gx, gy, -0.2});
      ground.normals().push_back({0.0, 0.0, 1.0});
    }
  ground.tris().push_back({0, 1, 3});
  ground.tris().push_back({0, 3, 2});
  auto g = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("ground");
  g->setGeometry(ground);
  g->setColor(0.35, 0.35, 0.35);
  g->setAmbient(0.3);
  g->setDiffuse(0.7);
  const int sun = sg.addDirectionalLight(135.0, 60.0);

  SceneRenderer sr(sg, W, H, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  topView(sr, C, C, 100);
  check(sg.setShadowsEnabled(true), "shadows enabled");
  sr.render();
  glcountInstall();

  for (int fr = 1; fr <= 3; ++fr) { // one append, one clip step, one link move per frame
    const GLCount g0 = glcountRead();
    const cvc::gl::StreamStats s0 = track->streamStats();
    const int k = 39 + fr;
    // Re-aim the sun a hair so VTK's baker re-bakes THIS frame: streamed writes
    // bump no MTime, so on their own they never trigger a bake (and
    // invalidateShadowBake() only lifts cvcGL's interval, not VTK's own check).
    sg.setLightDirection(sun, 135.0 + 0.01 * fr, 60.0);
    track->append(static_cast<float>(C - 80 + 3 * k), static_cast<float>(C - 30), 0.5f);
    spine->setVisibleCenters(5.0 * fr + 0.5);
    link->setEndpoints(static_cast<float>(C - 60), static_cast<float>(C + 70 + fr),
                       static_cast<float>(C + 60), static_cast<float>(C + 70 + fr));
    sr.render();
    const GLCount d = glcountRead() - g0;
    const cvc::gl::StreamStats s1 = track->streamStats();
    check(d.uploadCalls() == 1 && d.uploadBytes() == 48 && d.createdNothing() &&
              s1.uploads - s0.uploads == 1 && d.v[K_DRAWS] >= 8,
          "frame " + std::to_string(fr) + ": bake + shadow pass, one 48-byte upload",
          glcountStr(d));
  }
  const Frame f = grab(sr);
  check(hueNear(f, toPx(sr, C - 80 + 3 * 20, C - 30, 0.5), 2, 0) &&
            !hueNear(f, toPx(sr, C - 80 + 3 * 2, C - 30, 0.5), 2, 0),
        "track sub-range drawn");
  check(!hueNear(f, toPx(sr, C - 80 + 3 * 5, C + 30, 0.5), 2, 1) &&
            hueNear(f, toPx(sr, C - 80 + 3 * 30, C + 30, 0.5), 2, 1),
        "spine clipped");
  double p[3];
  link->centerAt(0.75, p);
  check(hueNear(f, toPx(sr, p[0], p[1], p[2]), 3, 2), "link drawn");
  check(ErrorCounter::errors() == errors0, "no VTK errors (shaders compile in every pass)",
        std::to_string(ErrorCounter::errors() - errors0));
}

} // namespace

int main() {
  disableSwapThrottle();
  installErrorCounter();
  cvc::app app;
  if (renderAvailable(app)) {
    {
      SceneGraph sg(app, "gl_probe");
      SceneRenderer sr(sg, 16, 16, /*offscreen=*/true);
      sr.render();
      printRenderer(sr);
    }
    for (StreamingMapperKind k : {StreamingMapperKind::Classic, StreamingMapperKind::LowMemory}) {
      testTrack(app, k);
      testSpine(app, k);
      testLink(app, k);
      testLinkClip(app, k);
      testLinkTerrainLater(app, k);
      testTrackLeavesBox(app, k);
      testPicking(app, k);
      testShadows(app, k);
    }
  }
  check(ErrorCounter::errors() == 0, "no VTK errors overall",
        std::to_string(ErrorCounter::errors()));
  return finish("cvcgl_streaming_overlays");
}
